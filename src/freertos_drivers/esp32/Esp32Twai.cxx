/** \copyright
 * Copyright (c) 2020, Mike Dunston
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file Esp32Twai.hxx
 *
 * TWAI driver implementation for OpenMRN. This leverages a clone the ESP-IDF
 * HAL API rather than the TWAI driver to allow for faster access to received
 * frames and support for ESP-IDF v4.0 and above.
 *
 * @author Mike Dunston
 * @date 24 September 2020
 */

#include "freertos_drivers/esp32/Esp32Twai.hxx"

#ifdef ESP32

#include <driver/gpio.h>
#include <driver/periph_ctrl.h>
#include <esp_task.h>
#include <esp_vfs.h>
#include <esp_intr_alloc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#if __has_include(<esp_idf_version.h>)
#include <esp_idf_version.h>
#else
#include <esp_system.h>

#ifndef ESP_IDF_VERSION
#define ESP_IDF_VERSION 0
#endif

#ifndef ESP_IDF_VERSION_VAL
#define ESP_IDF_VERSION_VAL(a,b,c) 1
#endif

#endif

#include "can_frame.h"
#include "can_ioctl.h"
#include "Esp32TwaiHal.hxx"
#include "executor/Notifiable.hxx"
#include "freertos_drivers/arduino/DeviceBuffer.hxx"
#include "nmranet_config.h"
#include "utils/Atomic.hxx"
#include "utils/constants.hxx"
#include "utils/logging.h"

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4,0,0)
/// Data type used internally by ESP-IDF select() calls.
typedef SemaphoreHandle_t * esp_vfs_select_sem_t;
#endif // IDF < 4.0

/// Default file descriptor to return in the vfs_open() call.
///
/// NOTE: The TWAI driver only supports one file descriptor at this time.
static constexpr int TWAI_VFS_FD = 0;

/// Interval at which to print the ESP32 TWAI bus status.
static constexpr TickType_t STATUS_PRINT_INTERVAL = pdMS_TO_TICKS(10000);

/// TWAI peripheral ISR flags.
///
/// Since the ISR is written in C/C++ code we can only specify the flag
/// ESP_INTR_FLAG_LOWMED. We can optionally specify ESP_INTR_FLAG_IRAM but
/// there are intermittent crashes when the ISR is in IRAM due to flash access
/// (ie SPIFFS or LittleFS).
static constexpr int TWAI_ISR_FLAGS = ESP_INTR_FLAG_LOWMED;

// Starting in IDF v4.2 the CAN peripheral was renamed to TWAI.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,2,0)
/// Pin matrix index for the TWAI TX peripheral pin.
static constexpr uint32_t TWAI_TX_SIGNAL_IDX = TWAI_TX_IDX;

/// Pin matrix index for the TWAI RX peripheral pin.
static constexpr uint32_t TWAI_RX_SIGNAL_IDX = TWAI_RX_IDX;

/// TWAI peripheral ISR source index.
static constexpr int TWAI_ISR_SOURCE = ETS_TWAI_INTR_SOURCE;

/// TWAI Peripheral module ID.
static constexpr periph_module_t TWAI_PERIPHERAL = PERIPH_TWAI_MODULE;
#else
/// Pin matrix index for the TWAI TX peripheral pin.
static constexpr uint32_t TWAI_TX_SIGNAL_IDX = CAN_TX_IDX;

/// Pin matrix index for the TWAI RX peripheral pin.
static constexpr uint32_t TWAI_RX_SIGNAL_IDX = CAN_RX_IDX;

/// TWAI peripheral ISR source index.
static constexpr int TWAI_ISR_SOURCE = ETS_CAN_INTR_SOURCE;

/// TWAI Peripheral module ID.
static constexpr periph_module_t TWAI_PERIPHERAL = PERIPH_CAN_MODULE;
#endif // ESP_IDF_VERSION > 4.2.0

/// Internal flag used for tracking if the VFS driver has been registered.
static bool vfs_is_registered = false;

/// Internal flag used for tracking if the low-level TWAI driver has been
/// configured.
static bool twai_is_configured = false;

/// TWAI default interrupt enable mask, excludes data overrun (bit[3]) and
/// brp_div (bit[4]) since these are not supported on all models.
static constexpr uint32_t TWAI_DEFAULT_INTERRUPTS = 0xE7;

/// TWAI HAL context object.
static twai_hal_context_t twai_context;

/// Handle for the TWAI ISR.
static intr_handle_t twai_isr_handle;

/// TWAI TX queue handle.
static QueueHandle_t twai_tx_queue_handle;

/// TWAI RX queue handle.
static QueueHandle_t twai_rx_queue_handle;

/// TWAI driver runtime statistics holders.
typedef struct 
{
    /// Number of frames that are pending transmission by the low-level TWAI
    /// driver.
    /// NOTE: This saves calling FreeRTOS functions to check @ref twai_tx_queue.
    uint32_t tx_pending;

    /// Number of frames that have been received by the low-level TWAI driver but
    /// have not been read from @ref twai_rx_queue.
    /// NOTE: This saves calling FreeRTOS functions to check @ref twai_rx_queue.
    uint32_t rx_pending;

    /// Number of frames have been removed from @ref twai_rx_queue and sent to the
    /// OpenMRN stack.
    uint32_t rx_processed;

    /// Number of frames frames that could not be sent to @ref twai_rx_queue.
    uint32_t rx_overrun_count;

    /// Number of frames that were discarded that had too large of a DLC count.
    uint32_t rx_discard_count;

    /// Number of frames that have been sent to the @ref twai_tx_queue by the
    /// OpenMRN stack successfully.
    uint32_t tx_processed;

    /// Number of frames that have been transmitted successfully by the low-level
    /// TWAI driver.
    uint32_t tx_success_count;

    /// Number of frames that have been could not be transmitted successfully by
    /// the low-level TWAI driver.
    uint32_t tx_failed_count;

    /// Number of arbitration errors that have been observed on the TWAI bus.
    uint32_t arb_lost_count;

    /// Number of general bus errors that have been observed on the TWAI bus.
    uint32_t bus_error_count;
} twai_driver_stats;

/// Runtime statistics for the TWAI driver.
static twai_driver_stats twai_stats = {};

/// Atomic used to protect @ref twai_stats.
static Atomic twai_stats_lock;

/// Thread handle for the statistics reporting task.
static os_thread_t status_thread_handle;

/// Semaphore provided by the VFS layer when select() is invoked on one of the
/// file descriptors handle by the TWAI VFS adapter.
static esp_vfs_select_sem_t twai_select_sem;

/// Internal flag indicating that the VFS layer has called
/// @ref vfs_start_select() and that @ref twai_select_sem should be used if/when
/// there is a change in @ref twai_tx_queue_handle or @ref twai_rx_queue_handle.
static volatile uint32_t twai_select_pending;

/// Flushes all frames in @ref twai_tx_queue_handle.
static inline void twai_flush_tx_queue()
{
    AtomicHolder h(&twai_stats_lock);
    xQueueReset(twai_tx_queue_handle);
    twai_stats.tx_pending = 0;
}

/// Flushes all frames in @ref twai_rx_queue_handle.
static inline void twai_flush_rx_queue()
{
    AtomicHolder h(&twai_stats_lock);
    xQueueReset(twai_rx_queue_handle);
    twai_stats.rx_pending = 0;
}

/// VFS adapter for open(path, flags, mode).
///
/// @param path is the path to the file being opened.
/// @param flags are the flags to use for opened file.
/// @param mode is the mode to use for the opened file.
///
/// When this method is invoked it will enable the TWAI driver and start the
/// periodic timer used for RX/TX of frame data.
///
/// @return 0 upon success, -1 upon failure with errno containing the cause.
static int twai_vfs_open(const char *path, int flags, int mode)
{
    // skip past the '/' that is passed in as first character
    path++;

    LOG(INFO, "[TWAI] Enabling TWAI device:%s", path);
    twai_flush_tx_queue();
    twai_flush_rx_queue();

    twai_hal_start(&twai_context, TWAI_MODE_NORMAL);
    return TWAI_VFS_FD;
}

/// VFS adapter for close(fd).
///
/// @param path is the path to the file being opened.
/// @param flags are the flags to use for opened file.
/// @param mode is the mode to use for the opened file.
///
/// When this method is invoked it will disable the TWAI driver and stop the
/// periodic timer used for RX/TX of frame data if it is running.
///
/// @return zero upon success, negative value with errno for failure.
static int twai_vfs_close(int fd)
{
    LOG(INFO, "[TWAI] Disabling TWAI fd:%d", fd);
    twai_flush_tx_queue();

    twai_hal_stop(&twai_context);
    return 0;
}

/// VFS interface helper for write()
///
/// @param fd is the file descriptor being written to.
/// @param buf is the buffer containing the data to be written.
/// @param size is the size of the buffer.
/// @return number of bytes written or -1 if there is the write would be a
/// blocking operation.
static ssize_t twai_vfs_write(int fd, const void *buf, size_t size)
{
    DASSERT(fd == TWAI_VFS_FD);
    ssize_t sent = 0;
    const struct can_frame *data = (const struct can_frame *)buf;
    size /= sizeof(struct can_frame);
    while (size)
    {
        if (twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_BUS_OFF))
        {
            twai_hal_start_bus_recovery(&twai_context);
            twai_flush_tx_queue();
            break;
        }
        else if (!twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_RUNNING))
        {
            break;
        }

        twai_message_t tx_frame;
        twai_hal_frame_t hal_frame;
        bzero(&tx_frame, sizeof(twai_message_t));
        tx_frame.identifier = data->can_id;
        tx_frame.extd = data->can_eff;
        tx_frame.rtr = data->can_rtr;
        tx_frame.data_length_code = data->can_dlc;
        memcpy(tx_frame.data, data->data, data->can_dlc);
        twai_hal_format_frame(&tx_frame, &hal_frame);
        if (xQueueSend(twai_tx_queue_handle, &hal_frame, 0) == pdTRUE)
        {
            {
                AtomicHolder h(&twai_stats_lock);
                twai_stats.tx_pending++;
                twai_stats.tx_processed++;
            }
            // If the bus is running and the TX buffer is not occupied we can
            // start the TX from here.
            if (twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_RUNNING) &&
                !twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED) &&
                xQueueReceive(twai_tx_queue_handle, &hal_frame, 0) == pdTRUE)
            {
                twai_hal_set_tx_buffer_and_transmit(&twai_context, &hal_frame);
            }
            sent++;
            size--;
        }
        else
        {
            // No space in the TX queue
            break;
        }
    }
    if (!sent)
    {
        errno = EWOULDBLOCK;
        return -1;
    }
    return sent * sizeof(struct can_frame);
}

/// VFS interface helper for read()
///
/// @param fd is the file descriptor being read from.
/// @param buf is the buffer to write into.
/// @param size is the size of the buffer.
/// @return number of bytes read or -1 if there is the read would be a
/// blocking operation.
static ssize_t twai_vfs_read(int fd, void *buf, size_t size)
{
    DASSERT(fd == TWAI_VFS_FD);

    ssize_t received = 0;
    struct can_frame *data = (struct can_frame *)buf;
    size /= sizeof(struct can_frame);
    while (size)
    {
        twai_hal_frame_t hal_frame;
        memset(&hal_frame, 0, sizeof(twai_hal_frame_t));
        if (xQueueReceive(twai_rx_queue_handle, &hal_frame, 0) != pdTRUE)
        {
            // no frames available to receive
            break;
        }
        {
            AtomicHolder h(&twai_stats_lock);
            twai_stats.rx_pending--;
            twai_stats.rx_processed++;
        }
        twai_message_t rx_frame;
        twai_hal_parse_frame(&hal_frame, &rx_frame);
        memcpy(data->data, rx_frame.data, TWAI_FRAME_MAX_DLC);
        data->can_dlc = rx_frame.data_length_code;
        data->can_id = rx_frame.identifier;
        data->can_eff = rx_frame.extd;
        data->can_rtr = rx_frame.rtr;
        size--;
        received++;
        data++;
    }
    if (!received)
    {
        errno = EWOULDBLOCK;
        return -1;
    }

    return received * sizeof(struct can_frame);
}

/// VFS interface helper for select()
///
/// @param nfds is the number of FDs being checked.
/// @param readfds is the set of FDs being checked for ready to read.
/// @param writefds is the set of FDs being checked for ready to write.
/// @param exceptfds is the set of FDs being checked for exception.
/// @param sem is the semaphore to use for waking up the select() call.
/// @param end_select_args is any arguments to pass into vfs_end_select().
/// @return ESP_OK for success.
static esp_err_t twai_vfs_start_select(int nfds, fd_set *readfds
                                     , fd_set *writefds, fd_set *exceptfds
                                     , esp_vfs_select_sem_t sem
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,0,0)
                                     , void **end_select_args
#endif
)
{
    HASSERT(nfds >= 1);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,0,0)
    *end_select_args = nullptr;
#endif
    twai_select_sem = sem;
    twai_select_pending = 1;

    return ESP_OK;
}

/// VFS interface helper invoked when select() is woken up.
///
/// @param end_select_args is any arguments provided in vfs_start_select().
/// @return ESP_OK for success.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,0,0)
static esp_err_t twai_vfs_end_select(void *end_select_args)
#else
static void twai_vfs_end_select()
#endif
{
    twai_select_pending = 0;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,0,0)
    return ESP_OK;
#endif
}

/// VFS adapter for fcntl(fd, cmd, arg).
///
/// @param fd to operate on.
/// @param cmd to be executed.
/// @param arg arg to be used for the operation.
///
/// This method is currently a NO-OP.
///
/// @return zero upon success, negative value with errno for failure.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,0,0)
static int twai_vfs_fcntl(int fd, int cmd, int arg)
#else
static int twai_vfs_fcntl(int fd, int cmd, va_list arg)
#endif
{
    DASSERT(fd == TWAI_VFS_FD);

    // NO OP

    return 0;
}

/// Interrupt routine that processing events raised by the low-level TWAI
/// driver.
///
/// @param arg Unused
static IRAM_ATTR void twai_isr(void *arg)
{
    BaseType_t wakeup = pdFALSE;
    uint32_t events = twai_hal_decode_interrupt_events(&twai_context);
    bool is_rx = (events & TWAI_HAL_EVENT_RX_BUFF_FRAME);
    bool is_tx = (events & TWAI_HAL_EVENT_TX_BUFF_FREE);

    AtomicHolder h(&twai_stats_lock);

    if (is_rx)
    {
        uint32_t msg_count = twai_hal_get_rx_msg_count(&twai_context);
        for (int i = 0; i < msg_count; i++)
        {
            twai_hal_frame_t frame;
            twai_hal_read_rx_buffer_and_clear(&twai_context, &frame);
            if (frame.dlc > TWAI_FRAME_MAX_DLC)
            {
                twai_stats.rx_discard_count++;
            }
            else if (xQueueSendFromISR(twai_rx_queue_handle, &frame, &wakeup) == pdTRUE)
            {
                twai_stats.rx_pending++;
            }
            else
            {
                twai_stats.rx_overrun_count++;
            }
        }
    }

    if (is_tx)
    {
        twai_stats.tx_pending--;
        if (twai_hal_check_last_tx_successful(&twai_context))
        {
            twai_stats.tx_success_count++;
        }
        else
        {
            twai_stats.tx_failed_count++;
        }

        twai_hal_frame_t tx_frame;
        if (xQueueReceiveFromISR(twai_tx_queue_handle, &tx_frame, &wakeup) == pdTRUE)
        {
            twai_hal_set_tx_buffer_and_transmit(&twai_context, &tx_frame);
        }
    }

    if (events & TWAI_HAL_EVENT_BUS_RECOV_CPLT)
    {
        twai_hal_start(&twai_context, TWAI_MODE_NORMAL);
    }
    if (events & TWAI_HAL_EVENT_BUS_ERR)
    {
        twai_stats.bus_error_count++;
    }
    if (events & TWAI_HAL_EVENT_ARB_LOST)
    {
        twai_stats.arb_lost_count++;
    }

    if ((is_rx || is_tx) && twai_select_pending)
    {
        esp_vfs_select_triggered_isr(twai_select_sem, &wakeup);
    }


// In IDF v4.3 the portYIELD_FROM_ISR macro was extended to take the value of
// the wakeup flag and if true will yield as expected otherwise it is mostly a
// no-op.
#if ESP_IDF_VERSION <= ESP_IDF_VERSION_VAL(4,2,0)
    if (wakeup == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
#else
    portYIELD_FROM_ISR(wakeup);
#endif // ESP_IDF_VERSION > 4.2.0
}

/// Periodic TWAI statistics reporting task.
///
/// @param param (unused)
static void *report_stats(void *param)
{
    while (twai_is_configured)
    {
        // local copy of the stats for reporting.
        twai_driver_stats stats;
        {
            AtomicHolder h(&twai_stats_lock);
            stats = twai_stats;
        }

        LOG(INFO
          , "[TWAI] RX:%d (pending:%d,overrun:%d,discard:%d)"
            " TX:%d (pending:%d,suc:%d,fail:%d)"
            " bus (arb-err:%d,err:%d,state:%s)"
          , stats.rx_processed, stats.rx_pending, stats.rx_overrun_count
          , stats.rx_discard_count, stats.tx_processed, stats.tx_pending
          , stats.tx_success_count, stats.tx_failed_count
          , stats.arb_lost_count, stats.bus_error_count
          , twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_RUNNING) ? "Running"
          : twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_RECOVERING) ? "Recovering"
          : twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_ERR_WARN) ? "Err-Warn"
          : twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_ERR_PASSIVE) ? "Err-Pasv"
          : "Bus Off");

        // delay until the next reporting interval, this is being used instead
        // of vTaskDelay to allow early wake up in the case of shutdown of the
        // TWAI driver.
        ulTaskNotifyTake(pdTRUE, STATUS_PRINT_INTERVAL);
    }
    return nullptr;
}

static inline void configure_twai_gpio(gpio_num_t tx, gpio_num_t rx)
{
    LOG(VERBOSE, "[TWAI] Configuring TWAI TX pin: %d", tx);
    gpio_set_pull_mode(tx, GPIO_FLOATING);
    gpio_matrix_out(tx, TWAI_TX_SIGNAL_IDX, false, false);
    gpio_pad_select_gpio(tx);

    LOG(VERBOSE, "[TWAI] Configuring TWAI RX pin: %d", rx);
    gpio_set_pull_mode(rx, GPIO_FLOATING);
    gpio_set_direction(rx, GPIO_MODE_INPUT);
    gpio_matrix_in(rx, TWAI_RX_SIGNAL_IDX, false);
    gpio_pad_select_gpio(rx);
}

static inline void create_twai_buffers()
{
    LOG(VERBOSE, "[TWAI] Creating TWAI TX queue: %d"
      , config_can_tx_buffer_size());

    twai_tx_queue_handle =
        xQueueCreate(config_can_tx_buffer_size(), sizeof(twai_hal_frame_t));
    HASSERT(twai_tx_queue_handle != nullptr);

    LOG(VERBOSE, "[TWAI] Creating TWAI RX queue: %d"
      , config_can_rx_buffer_size());
    twai_rx_queue_handle =
        xQueueCreate(config_can_rx_buffer_size(), sizeof(twai_hal_frame_t));
    HASSERT(twai_rx_queue_handle != nullptr);
}

static inline void initialize_twai()
{
    periph_module_reset(TWAI_PERIPHERAL);
    periph_module_enable(TWAI_PERIPHERAL);
    HASSERT(twai_hal_init(&twai_context));
    twai_timing_config_t timingCfg = TWAI_TIMING_CONFIG_125KBITS();
    twai_filter_config_t filterCfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    LOG(VERBOSE, "[TWAI] Initiailizing TWAI peripheral");
    twai_hal_configure(&twai_context, &timingCfg, &filterCfg
                     , TWAI_DEFAULT_INTERRUPTS, 0);
    LOG(VERBOSE, "[TWAI] Allocating TWAI ISR");
    ESP_ERROR_CHECK(
        esp_intr_alloc(TWAI_ISR_SOURCE, TWAI_ISR_FLAGS, twai_isr, NULL
                     , &twai_isr_handle));
}

// Constructor.
Esp32Twai::Esp32Twai(const char *vfsPath, int rxPin, int txPin, bool report)
    : vfsPath_(vfsPath), rxPin_((gpio_num_t)rxPin), txPin_((gpio_num_t)txPin)
    , reportStats_(report)
{
    HASSERT(GPIO_IS_VALID_GPIO(rxPin));
    HASSERT(GPIO_IS_VALID_OUTPUT_GPIO(txPin));
}

// Destructor.
Esp32Twai::~Esp32Twai()
{
    if (vfs_is_registered)
    {
        ESP_ERROR_CHECK(esp_vfs_unregister(vfsPath_));
    }

    if (twai_is_configured)
    {
        ESP_ERROR_CHECK(esp_intr_free(twai_isr_handle));
        twai_hal_deinit(&twai_context);
        twai_is_configured = false;
        xTaskNotifyGive(status_thread_handle);
        vQueueDelete(twai_tx_queue_handle);
        vQueueDelete(twai_rx_queue_handle);
    }
}

// Initializes the VFS adapter and TWAI driver
void Esp32Twai::hw_init()
{
    esp_vfs_t vfs;
    memset(&vfs, 0, sizeof(esp_vfs_t));
    vfs.write = twai_vfs_write;
    vfs.read = twai_vfs_read;
    vfs.open = twai_vfs_open;
    vfs.close = twai_vfs_close;
    vfs.fcntl = twai_vfs_fcntl;
    vfs.start_select = twai_vfs_start_select;
    vfs.end_select = twai_vfs_end_select;
    vfs.flags = ESP_VFS_FLAG_DEFAULT;
    ESP_ERROR_CHECK(esp_vfs_register(vfsPath_, &vfs, this));
    vfs_is_registered = true;

    configure_twai_gpio(txPin_, rxPin_);
    create_twai_buffers();
    initialize_twai();

    twai_is_configured = true;
    if (reportStats_)
    {
        os_thread_create(&status_thread_handle, "TWAI-STATS", 0, 0,
                         report_stats, nullptr);
    }
}

#endif // ESP32