/** \copyright
 * Copyright (c) 2021, Mike Dunston
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
 * \file Esp32HardwareTwai.cxx
 *
 * TWAI driver implementation for OpenMRN. This leverages the ESP-IDF TWAI HAL
 * API rather than the TWAI driver to allow for a more integrated solution than
 * the TWAI driver which requires polling for RX. This implementation supports
 * both ::select and the non-blocking ::ioctl/::fnctl approach.
 *
 * @author Mike Dunston
 * @date 1 May 2021
 */

// Ensure we only compile this code for the ESP32 family of MCUs and that the
// ESP-IDF version is supported for this code.
#if defined(ESP32)

#include "sdkconfig.h"
#include <esp_idf_version.h>

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,3,0)

#ifdef CONFIG_VFS_SUPPORT_TERMIOS
// remove defines added by arduino-esp32 core/esp32/binary.h which are
// duplicated in sys/termios.h which may be included by esp_vfs.h
#undef B110
#undef B1000000
#endif // CONFIG_VFS_SUPPORT_TERMIOS

#include <assert.h>
#include <driver/gpio.h>
#include <driver/periph_ctrl.h>
#include <esp_rom_gpio.h>
#include <esp_intr_alloc.h>
#include <esp_vfs.h>
#include <fcntl.h>
#include <hal/twai_types.h>
#include <hal/twai_hal.h>

#include "can_frame.h"
#include "can_ioctl.h"
#include "executor/Notifiable.hxx"
#include "freertos_drivers/arduino/DeviceBuffer.hxx"
#include "freertos_drivers/esp32/Esp32HardwareTwai.hxx"
#include "utils/Atomic.hxx"
#include "utils/logging.h"

namespace openmrn_arduino
{

/// Default file descriptor to return in the open() call.
///
/// NOTE: The TWAI driver only supports one file descriptor at this time.
static constexpr int TWAI_VFS_FD = 0;

/// Interval at which to print the ESP32 TWAI bus status.
static constexpr TickType_t STATUS_PRINT_INTERVAL = pdMS_TO_TICKS(10000);

/// TWAI default interrupt enable mask, excludes data overrun (bit[3]) and
/// brp_div (bit[4]) since these are not supported on all models.
static constexpr uint32_t TWAI_DEFAULT_INTERRUPTS = 0xE7;

/// TWAI Driver statistics.
typedef struct
{
    /// Number of frames have been removed from @ref rxBuf_ and sent to the
    /// OpenMRN stack.
    uint32_t rx_processed;

    /// Number of frames frames that could not be sent to @ref rx_buf.
    uint32_t rx_missed;

    /// Number of frames that were discarded that had too large of a DLC count.
    uint32_t rx_discard;

    /// Number of frames that were lost due to driver reset.
    uint32_t rx_lost;

    /// Number of frames that were lost due to RX FIFO overrun.
    uint32_t rx_overrun;

    /// Number of frames that have been sent to the @ref twai_tx_queue by the
    /// OpenMRN stack successfully.
    uint32_t tx_processed;

    /// Number of frames that have been transmitted successfully by the
    /// low-level TWAI driver.
    uint32_t tx_success;

    /// Number of frames that have been could not be transmitted successfully
    /// by the low-level TWAI driver.
    uint32_t tx_failed;

    /// Number of arbitration errors that have been observed on the TWAI bus.
    uint32_t arb_error;

    /// Number of general bus errors that have been observed on the TWAI bus.
    uint32_t bus_error;

    /// TWAI HAL context object.
    twai_hal_context_t hal_context;

    /// Handle for the TWAI ISR.
    intr_handle_t isr_handle;

    /// Transmit buffer
    DeviceBuffer<struct can_frame> *tx_buf;

    /// Receive buffer
    DeviceBuffer<struct can_frame> *rx_buf;

    /// Lock protecting @ref tx_buf and @ref rx_buf.
    Atomic buf_lock;

    /// This will be notified if the device has data avilable for read.
    Notifiable* readable_notify;

    /// This will be notified if the device has buffer avilable for write.
    Notifiable* writable_notify;

    /// Flag indicating that the file descriptor has been opened with the
    /// O_NONBLOCK flag.
    bool non_blocking;

    /// VFS semaphore that can be used to prematurely wakeup a call to select.
    /// NOTE: This is only valid after VFS has called @ref start_select and is
    /// invalid after VFS calls @ref end_select.
    esp_vfs_select_sem_t select_sem;

    /// Flag to indicate that @ref selectSem_ is valid or not.
    bool pending_select;

    /// Pointer to the fd_set provided by the ESP32 VFS layer used to indicate
    /// the fd is ready to be read.
    fd_set *read_fds;

    /// Copy of the fd_set provided by the ESP32 VFS layer used to check if
    /// the there is a read operation pending for the fd.
    fd_set orig_read_fds;

    /// Copy of the fd_set provided by the ESP32 VFS layer used to indicate the
    /// fd is ready to be written to.
    fd_set *write_fds;

    /// Copy of the fd_set provided by the ESP32 VFS layer used to check if
    /// the there is a write operation pending for the fd.
    fd_set orig_write_fds;

    /// Internal flag used for tracking if the VFS driver has been registered.
    bool vfs = false;

    /// Internal flag used for tracking if the low-level TWAI driver has been
    /// configured.
    bool configured = false;

} TwaiDriver;

/// TWAI Driver statistics instance.
static TwaiDriver twai;

/// VFS adapter for write(fd, buf, size)
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
        if (twai_hal_check_state_flags(&twai.hal_context, TWAI_HAL_STATE_FLAG_BUS_OFF))
        {
            twai_hal_start_bus_recovery(&twai.hal_context);
            twai.tx_buf->flush();
            break;
        }
        else if (!twai_hal_check_state_flags(&twai.hal_context, TWAI_HAL_STATE_FLAG_RUNNING))
        {
            break;
        }
        
        size_t frames_written = 0;
        {
            AtomicHolder h(&twai.buf_lock);
            frames_written = twai.tx_buf->put(data, size < 8 ? size : 8);
        }
        if (frames_written == 0)
        {
            // No space in the TX queue
            break;
        }
        if (twai_hal_check_state_flags(&twai.hal_context, TWAI_HAL_STATE_FLAG_RUNNING) &&
            !twai_hal_check_state_flags(&twai.hal_context, TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED))
        {
            // since the TX buffer is not occupied, retrieve the first
            // frame and transmit it here.
            AtomicHolder h(&twai.buf_lock);
            struct can_frame frame;
            twai_message_t tx_frame;
            twai_hal_frame_t hal_frame;
            HASSERT(twai.tx_buf->get(&frame, 1));
            memset(&tx_frame, 0, sizeof(twai_message_t));
            tx_frame.identifier = frame.can_id;
            tx_frame.extd = frame.can_eff;
            tx_frame.rtr = frame.can_rtr;
            tx_frame.data_length_code = frame.can_dlc;
            memcpy(tx_frame.data, frame.data, frame.can_dlc);
            twai_hal_format_frame(&tx_frame, &hal_frame);
            twai_hal_set_tx_buffer_and_transmit(&twai.hal_context, &hal_frame);
        }
        sent += frames_written;
        size -= frames_written;
    }
    if (!sent)
    {
        if (twai.non_blocking)
        {
            return 0;
        }
        errno = EWOULDBLOCK;
        return -1;
    }
    return sent * sizeof(struct can_frame);
}

/// VFS adapter for read(fd, buf, size)
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
        size_t received_frames = 0;
        {
            AtomicHolder h(&twai.buf_lock);
            received_frames = twai.rx_buf->get(data, size < 8 ? size : 8);
        }
        if (received_frames == 0)
        {
            break;
        }
        twai.rx_processed += received_frames;
        size -= received_frames;
        received += received_frames;
        data += received_frames;
    }
    if (!received)
    {
        errno = EWOULDBLOCK;
        return -1;
    }

    return received * sizeof(struct can_frame);
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
    twai.non_blocking = (flags & O_NONBLOCK);

    LOG(INFO, "[TWAI] Enabling TWAI device:%s mode:%x (%s)", path, mode,
        twai.non_blocking ? "non-blocking" : "blocking");
    {
        AtomicHolder h(&twai.buf_lock);
        twai.tx_buf->flush();
        twai.rx_buf->flush();
    }

    twai_hal_start(&twai.hal_context, TWAI_MODE_NORMAL);
    return TWAI_VFS_FD;
}

/// VFS adapter for close(fd).
///
/// @param fd is the file descriptor to close.
///
/// When this method is invoked it will disable the TWAI driver and stop the
/// periodic timer used for RX/TX of frame data if it is running.
///
/// @return zero upon success, negative value with errno for failure.
static int twai_vfs_close(int fd)
{
    LOG(INFO, "[TWAI] Disabling TWAI fd:%d", fd);
    twai.tx_buf->flush();

    twai_hal_stop(&twai.hal_context);
    return 0;
}

/// VFS adapter for ioctl.
///
/// @param fd is the file descriptor to operate on.
/// @param cmd is the command to execute.
/// @param args is the args for the command.
///
/// @return zero upon success, negative value with errno for failure.
static int twai_vfs_ioctl(int fd, int cmd, va_list args)
{
    /* sanity check to be sure we have a valid key for this device */
    HASSERT(IOC_TYPE(cmd) == CAN_IOC_MAGIC);

    // Will be called at the end if non-null.
    Notifiable* n = nullptr;

    if (IOC_SIZE(cmd) == NOTIFIABLE_TYPE)
    {
        n = reinterpret_cast<Notifiable*>(va_arg(args, uintptr_t));
    }

    switch (cmd)
    {
        default:
            return -EINVAL;
        case CAN_IOC_READ_ACTIVE:
            {
                AtomicHolder h(&twai.buf_lock);
                if (!twai.rx_buf->pending())
                {
                    std::swap(n, twai.readable_notify);
                }
            }
            break;
        case CAN_IOC_WRITE_ACTIVE:
            {
                AtomicHolder h(&twai.buf_lock);
                if (!twai.tx_buf->space())
                {
                    std::swap(n, twai.writable_notify);
                }
            }
            break;
    }
    if (n)
    {
        n->notify();
    }
    return 0;
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
static int twai_vfs_fcntl(int fd, int cmd, int arg)
{
    HASSERT(fd == TWAI_VFS_FD);
    int result = 0;

    if (cmd == F_GETFL)
    {
        if (twai.non_blocking)
        {
            result |= O_NONBLOCK;
        }
    }
    else if (cmd == F_SETFL)
    {
        twai.non_blocking = arg & O_NONBLOCK;
    }
    else
    {
        errno = ENOSYS;
        result = -1;
    }

    return result;
}

/// VFS adapter for select()
///
/// @param nfds is the number of FDs being checked.
/// @param readfds is the set of FDs being checked for ready to read.
/// @param writefds is the set of FDs being checked for ready to write.
/// @param exceptfds is the set of FDs being checked for exception.
/// @param sem is the semaphore to use for waking up the select() call.
static esp_err_t twai_vfs_start_select(int nfds, fd_set *readfds,
                                       fd_set *writefds, fd_set *exceptfds,
                                       esp_vfs_select_sem_t sem,
                                       void **end_select_args)
{
    // If the TWAI FD is present in any of the FD sets we should process the
    // select call.
    if (nfds >= 1 &&
        (FD_ISSET(TWAI_VFS_FD, readfds) || FD_ISSET(TWAI_VFS_FD, writefds) ||
         FD_ISSET(TWAI_VFS_FD, exceptfds)))
    {
        twai.select_sem = sem;
        twai.read_fds = readfds;
        twai.orig_read_fds = *readfds;
        twai.write_fds = writefds;
        twai.orig_write_fds = *writefds;
        twai.pending_select = true;
    }
    else
    {
        twai.pending_select = false;
    }
    return ESP_OK;
}

/// VFS interface helper invoked when select() is woken up.
///
/// @param end_select_args is any arguments provided in vfs_start_select().
static esp_err_t twai_vfs_end_select(void *end_select_args)
{
    twai.pending_select = false;
    return ESP_OK;
}

/// TWAI Interrupt handler for receiving one (or more) TWAI frames.
static inline uint32_t twai_rx_frames()
{
    AtomicHolder h(&twai.buf_lock);
    uint32_t frames_ready = twai_hal_get_rx_msg_count(&twai.hal_context);
    struct can_frame *can_frame = nullptr;
    uint32_t rx_count = 0;
    for (uint32_t idx = 0; idx < frames_ready; idx++)
    {
        twai_hal_frame_t frame;
        if (twai_hal_read_rx_buffer_and_clear(&twai.hal_context, &frame))
        {
            if (frame.dlc > TWAI_FRAME_MAX_DLC)
            {
                // DLC is longer than supported, discard the frame.
                twai.rx_discard++;
            }
            else
            {
                if (twai.rx_buf->data_write_pointer(&can_frame))
                {
                    twai_message_t rx_frame;
                    twai_hal_parse_frame(&frame, &rx_frame);
                    memcpy(can_frame->data, rx_frame.data, TWAI_FRAME_MAX_DLC);
                    can_frame->can_dlc = rx_frame.data_length_code;
                    can_frame->can_id = rx_frame.identifier;
                    can_frame->can_eff = rx_frame.extd;
                    can_frame->can_rtr = rx_frame.rtr;
                    rx_count += twai.rx_buf->advance(1);
                }
                else
                {
                    twai.rx_missed++;
                }
            }
        }
        else
        {
            twai.rx_overrun++;
// If the SOC does not support automatic clearing of the RX FIFO we need to
// handle it here and break out of the loop.
#ifndef SOC_TWAI_SUPPORTS_RX_STATUS
            twai_hal_clear_rx_fifo_overrun(&twai.hal_context);
            break;
#endif // SOC_TWAI_SUPPORTS_RX_STATUS
        }
    }

    return rx_count;
}

/// TWAI Interrupt handler for sending a TWAI frame to the transmit buffer.
static inline uint32_t twai_tx_frame()
{
    AtomicHolder h(&twai.buf_lock);
    if (twai_hal_check_last_tx_successful(&twai.hal_context))
    {
        twai.tx_success++;
    }
    else
    {
        twai.tx_failed++;
    }

    // Check if we have a pending frame to transmit in the queue
    struct can_frame *can_frame = nullptr;
    if (twai.tx_buf->data_read_pointer(&can_frame))
    {
        twai_message_t tx_frame;
        twai_hal_frame_t hal_frame;
        memset(&tx_frame, 0, sizeof(twai_message_t));
        tx_frame.identifier = can_frame->can_id;
        tx_frame.extd = can_frame->can_eff;
        tx_frame.rtr = can_frame->can_rtr;
        tx_frame.data_length_code = can_frame->can_dlc;
        memcpy(tx_frame.data, can_frame->data, can_frame->can_dlc);
        twai_hal_format_frame(&tx_frame, &hal_frame);
        twai_hal_set_tx_buffer_and_transmit(&twai.hal_context, &hal_frame);
        return twai.tx_buf->consume(1);
    }
    return 0;
}

/// Interrupt handler for the TWAI device.
///
/// @param arg unused.
static void twai_isr(void *arg)
{
    bool wakeup_select = false;
    uint32_t events = twai_hal_get_events(&twai.hal_context);
    
#if defined(CONFIG_TWAI_ERRATA_FIX_RX_FRAME_INVALID) || \
    defined(CONFIG_TWAI_ERRATA_FIX_RX_FIFO_CORRUPT)
    if (events & TWAI_HAL_EVENT_NEED_PERIPH_RESET)
    {
        twai_hal_prepare_for_reset(&twai.hal_context);
        periph_module_reset(PERIPH_TWAI_MODULE);
        twai_hal_recover_from_reset(&twai.hal_context);
        twai.rx_lost += twai_hal_get_reset_lost_rx_cnt(&twai.hal_context);
    }
#endif

    // RX completed
    if ((events & TWAI_HAL_EVENT_RX_BUFF_FRAME) && twai_rx_frames())
    {
        wakeup_select = true;
        // std::swap is not ISR safe so it is not used here. We also
        // have a lock on the buffer at this point so it is safe to
        // directly access and reset this pointer.
        if (twai.readable_notify)
        {
            twai.readable_notify->notify_from_isr();
            twai.readable_notify = nullptr;
        }
    }

    // TX completed
    if ((events & TWAI_HAL_EVENT_TX_BUFF_FREE) && twai_tx_frame())
    {
        wakeup_select = true;
        // std::swap is not ISR safe so it is not used here. We also have a
        // lock on the buffer at this point so it is safe to directly
        // access and reset this pointer.
        if (twai.writable_notify)
        {
            twai.writable_notify->notify_from_isr();
            twai.writable_notify = nullptr;
        }
    }

    // Bus recovery complete, trigger a restart
    if (events & TWAI_HAL_EVENT_BUS_RECOV_CPLT)
    {
        // start the driver automatically
        twai_hal_start(&twai.hal_context, TWAI_MODE_NORMAL);
    }
    // Bus error detected
    if (events & TWAI_HAL_EVENT_BUS_ERR)
    {
        twai.bus_error++;
    }
    // Arbitration error detected
    if (events & TWAI_HAL_EVENT_ARB_LOST)
    {
        twai.arb_error++;
    }

    // If we need to wake up select() do so now.
    if (wakeup_select && twai.pending_select)
    {
        BaseType_t wakeup = pdFALSE;
        if (FD_ISSET(TWAI_VFS_FD, &twai.orig_read_fds) &&
            (events & TWAI_HAL_EVENT_RX_BUFF_FRAME))
        {
            FD_SET(TWAI_VFS_FD, twai.read_fds);
        }
        if (FD_ISSET(TWAI_VFS_FD, &twai.orig_write_fds) &&
            (events & TWAI_HAL_EVENT_TX_BUFF_FREE))
        {
            FD_SET(TWAI_VFS_FD, twai.write_fds);
        }
        esp_vfs_select_triggered_isr(twai.select_sem, &wakeup);
        if (wakeup == pdTRUE)
        {
            portYIELD_FROM_ISR();
        }
    }
}

/// Periodic TWAI statistics reporting task.
void* twai_stats_report(void* param)
{
    while (twai.configured)
    {
        LOG(INFO,
          "[TWAI] RX:%d (pending:%zu,overrun:%d,discard:%d,missed:%d,lost:%d)"
          " TX:%d (pending:%zu,suc:%d,fail:%d)"
          " bus (arb-err:%d,err:%d,state:%s)",
          twai.rx_processed, twai.rx_buf->pending(), twai.rx_overrun,
          twai.rx_discard, twai.rx_missed, twai.rx_lost,
          twai.tx_processed, twai.tx_buf->pending(), twai.tx_success,
          twai.tx_failed, twai.arb_error, twai.bus_error,
          twai_hal_check_state_flags(&twai.hal_context, TWAI_HAL_STATE_FLAG_RUNNING) ? "Running" :
          twai_hal_check_state_flags(&twai.hal_context, TWAI_HAL_STATE_FLAG_RECOVERING) ? "Recovering" :
          twai_hal_check_state_flags(&twai.hal_context, TWAI_HAL_STATE_FLAG_ERR_WARN) ? "Err-Warn" :
          twai_hal_check_state_flags(&twai.hal_context, TWAI_HAL_STATE_FLAG_ERR_PASSIVE) ? "Err-Pasv" :
          "Bus Off");

        // delay until the next reporting interval, this is being used instead
        // of vTaskDelay to allow early wake up in the case of shutdown of the
        // TWAI driver.
        ulTaskNotifyTake(pdTRUE, STATUS_PRINT_INTERVAL);
    }

    return NULL;
}

Esp32HardwareTwai::Esp32HardwareTwai(int rx, int tx, size_t rx_size,
                                     size_t tx_size, bool report,
                                     const char *path, int clock_out,
                                     int bus_status) :
                                     rxPin_(rx),
                                     txPin_(tx),
                                     extClockPin_(clock_out),
                                     busStatusPin_(bus_status),
                                     reportStats_(report), vfsPath_(path)
{
    HASSERT(GPIO_IS_VALID_GPIO(rxPin_));
    HASSERT(GPIO_IS_VALID_OUTPUT_GPIO(txPin_));

    if (extClockPin_ != GPIO_NUM_NC)
    {
        HASSERT(GPIO_IS_VALID_OUTPUT_GPIO(extClockPin_));
    }

    if (busStatusPin_ != GPIO_NUM_NC)
    {
        HASSERT(GPIO_IS_VALID_OUTPUT_GPIO(busStatusPin_));
    }

    memset(&twai, 0, sizeof(TwaiDriver));

    twai.rx_buf = DeviceBuffer<struct can_frame>::create(rx_size);
    HASSERT(twai.rx_buf != nullptr);

    twai.tx_buf =
        DeviceBuffer<struct can_frame>::create(tx_size, tx_size / 2);
    HASSERT(twai.tx_buf != nullptr);
}

Esp32HardwareTwai::~Esp32HardwareTwai()
{
    if (twai.configured)
    {
        ESP_ERROR_CHECK(esp_intr_free(twai.isr_handle));
        twai_hal_deinit(&twai.hal_context);
    }

    if (twai.vfs)
    {
        ESP_ERROR_CHECK(esp_vfs_unregister(vfsPath_));
    }

    twai.tx_buf->destroy();
    twai.rx_buf->destroy();
}

void Esp32HardwareTwai::hw_init()
{
    LOG(VERBOSE, "[TWAI] Configuring TX pin: %d", txPin_);
    gpio_set_pull_mode((gpio_num_t)txPin_, GPIO_FLOATING);
    esp_rom_gpio_connect_out_signal((gpio_num_t)txPin_, TWAI_TX_IDX, false,
        false);
    esp_rom_gpio_pad_select_gpio((gpio_num_t)txPin_);

    LOG(VERBOSE, "[TWAI] Configuring RX pin: %d", rxPin_);
    gpio_set_pull_mode((gpio_num_t)rxPin_, GPIO_FLOATING);
    esp_rom_gpio_connect_in_signal((gpio_num_t)rxPin_, TWAI_RX_IDX, false);
    esp_rom_gpio_pad_select_gpio((gpio_num_t)rxPin_);
    gpio_set_direction((gpio_num_t)rxPin_, GPIO_MODE_INPUT);

    if (extClockPin_ != GPIO_NUM_NC)
    {
        LOG(VERBOSE, "[TWAI] Configuring external clock pin: %d",
            extClockPin_);
        ESP_ERROR_CHECK(
            gpio_set_pull_mode((gpio_num_t)extClockPin_, GPIO_FLOATING));
        esp_rom_gpio_connect_out_signal((gpio_num_t)extClockPin_,
            TWAI_CLKOUT_IDX, false, false);
        esp_rom_gpio_pad_select_gpio((gpio_num_t)extClockPin_);
    }

    if (busStatusPin_ != GPIO_NUM_NC)
    {
        LOG(VERBOSE, "[TWAI] Configuring external bus status pin: %d",
            busStatusPin_);
        ESP_ERROR_CHECK(
            gpio_set_pull_mode((gpio_num_t)busStatusPin_, GPIO_FLOATING));
        esp_rom_gpio_connect_out_signal((gpio_num_t)busStatusPin_,
            TWAI_BUS_OFF_ON_IDX, false, false);
        esp_rom_gpio_pad_select_gpio((gpio_num_t)busStatusPin_);
    }

    esp_vfs_t vfs;
    memset(&vfs, 0, sizeof(esp_vfs_t));
    vfs.write = twai_vfs_write;
    vfs.read = twai_vfs_read;
    vfs.open = twai_vfs_open;
    vfs.close = twai_vfs_close;
    vfs.fcntl = twai_vfs_fcntl;
    vfs.ioctl = twai_vfs_ioctl;
    vfs.start_select = twai_vfs_start_select;
    vfs.end_select = twai_vfs_end_select;
    vfs.flags = ESP_VFS_FLAG_DEFAULT;
    ESP_ERROR_CHECK(esp_vfs_register(vfsPath_, &vfs, this));
    twai.vfs = true;

    periph_module_reset(PERIPH_TWAI_MODULE);
    periph_module_enable(PERIPH_TWAI_MODULE);
    HASSERT(twai_hal_init(&twai.hal_context));
    twai_timing_config_t timingCfg = TWAI_TIMING_CONFIG_125KBITS();
    twai_filter_config_t filterCfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    LOG(VERBOSE, "[TWAI] Initiailizing peripheral");
    twai_hal_configure(&twai.hal_context, &timingCfg, &filterCfg,
        TWAI_DEFAULT_INTERRUPTS, 0);
    LOG(VERBOSE, "[TWAI] Allocating ISR");
    ESP_ERROR_CHECK(
        esp_intr_alloc(ETS_TWAI_INTR_SOURCE, ESP_INTR_FLAG_LOWMED, twai_isr,
            this, &twai.isr_handle));
    twai.configured = true;

    if (reportStats_)
    {
        os_thread_create(nullptr, "TWAI-STAT", 0, 0, twai_stats_report, this);
    }
}

} // namespace openmrn_arduino

#endif // IDF v4.3+

#endif // ESP32
