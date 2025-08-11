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
#if defined(ESP_PLATFORM)

#include "sdkconfig.h"

#if CONFIG_VFS_SUPPORT_TERMIOS
// remove defines added by arduino-esp32 core/esp32/binary.h which are
// duplicated in sys/termios.h which may be included by esp_vfs.h
#undef B110
#undef B1000000
#endif // CONFIG_VFS_SUPPORT_TERMIOS

#include <assert.h>
#include <driver/gpio.h>
#include <esp_idf_version.h>

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,1,0)
#include <esp_clk_tree.h>
#endif
#include <esp_private/periph_ctrl.h>
#ifndef CONFIG_FREERTOS_UNICORE
#include <esp_ipc.h>
#endif
#include <esp_log.h>
#include <esp_rom_gpio.h>
#include <esp_intr_alloc.h>
#include <esp_task.h>
#include <esp_vfs.h>
#include <fcntl.h>
#include <hal/twai_types.h>
#include <hal/twai_hal.h>
#include <soc/gpio_sig_map.h>
#include <stdint.h>

#include "can_frame.h"
#include "can_ioctl.h"
#include "executor/Notifiable.hxx"
#include "freertos_drivers/common/DeviceBuffer.hxx"
#include "freertos_drivers/esp32/Esp32HardwareTwai.hxx"
#include "utils/Atomic.hxx"
#include "utils/logging.h"

namespace openmrn_arduino
{

/// Default file descriptor to return in the open() call.
///
/// NOTE: The TWAI driver only supports one file descriptor at this time.
static constexpr int TWAI_VFS_FD = 0;

/// Priority for the ESP32 TWAI status reporting task.
static constexpr BaseType_t WATCHDOG_TASK_PRIORITY = ESP_TASK_TCPIP_PRIO - 1;

/// Stack size (bytes) to use for the ESP32 TWAI status reporting task.
static constexpr BaseType_t WATCHDOG_TASK_STACK = 2548;

/// Interval at which to print the ESP32 TWAI bus status.
static constexpr TickType_t STATUS_PRINT_INTERVAL = pdMS_TO_TICKS(10000);

/// TWAI default interrupt enable mask, excludes data overrun (bit[3]) and
/// brp_div (bit[4]) since these are not supported on all models.
static constexpr uint32_t TWAI_DEFAULT_INTERRUPTS = 0xE7;

/// TWAI Driver ISR flags.
/// Defaults to level 1-3 (C/C++ compatible) and suspend when accessing flash.
static constexpr uint32_t TWAI_INTERRUPT_FLAGS = ESP_INTR_FLAG_LOWMED;

/// ESP-IDF LOG tag used for all TWAI driver log statements.
static constexpr const char *TWAI_LOG_TAG = "ESP-TWAI";

/// TWAI Driver State
typedef struct
{
    /// TWAI Driver statistics.
    esp32_twai_stats_t stats;

    /// TWAI HAL context object.
    twai_hal_context_t context;

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

#if CONFIG_VFS_SUPPORT_SELECT
    /// Lock protecting all VFS select() cached data.
    Atomic select_lock;

    /// VFS semaphore that can be used to prematurely wakeup a call to select.
    /// NOTE: This is only valid after VFS has called @ref start_select and is
    /// invalid after VFS calls @ref end_select.
    esp_vfs_select_sem_t select_sem;

    /// Pointer to the fd_set provided by the ESP32 VFS layer used to indicate
    /// the fd is ready to be read.
    fd_set *readfds;

    /// Copy of the fd_set provided by the ESP32 VFS layer used to check if
    /// the there is a read operation pending for the fd.
    fd_set readfds_orig;

    /// Pointer to the fd_set provided by the ESP32 VFS layer used to indicate
    /// the fd is ready to be written to.
    fd_set *writefds;

    /// Copy of the fd_set provided by the ESP32 VFS layer used to check if
    /// the fd is ready to be written to.
    fd_set writefds_orig;

    /// Pointer to the fd_set provided by the ESP32 VFS layer used to indicate
    /// the fd has an error.
    fd_set *exceptfds;

    /// Copy of the fd_set provided by the ESP32 VFS layer used to check if
    /// the fd has an error.
    fd_set exceptfds_orig;
#endif // CONFIG_VFS_SUPPORT_SELECT

    /// Internal flag used for tracking if the low-level TWAI driver has been
    /// configured and ready to use.
    bool active;

    /// Thread handle for the background thread that monitors the TWAI driver
    /// and periodically reports statistics.
    os_thread_t wd_thread;

    /// Internal flag used to enable the periodic printing of TWAI driver
    /// statistics.
    bool report_stats;
} TwaiDriver;

/// TWAI Driver statistics instance.
static TwaiDriver twai;

/// Helper function that will return true if the TWAI driver is currently in a
/// RUNNING state.
static inline bool is_twai_running()
{
    return twai_hal_check_state_flags(&twai.context, TWAI_HAL_STATE_FLAG_RUNNING);
}

/// Helper function that will return true if the TWAI driver is currently in a
/// RECOVERING state.
static inline bool is_twai_recovering()
{
    return twai_hal_check_state_flags(&twai.context, TWAI_HAL_STATE_FLAG_RECOVERING);
}

/// Helper function that will return true if the TWAI driver is currently in a
/// ERR-WARN state.
static inline bool is_twai_err_warn()
{
    return twai_hal_check_state_flags(&twai.context, TWAI_HAL_STATE_FLAG_ERR_WARN);
}

/// Helper function that will return true if the TWAI driver is currently in a
/// ERR-PASSIVE state.
static inline bool is_twai_err_passive()
{
    return twai_hal_check_state_flags(&twai.context, TWAI_HAL_STATE_FLAG_ERR_PASSIVE);
}

/// Helper function that will return true if the TWAI driver is currently in an
/// OFF state.
static inline bool is_twai_bus_off()
{
    return twai_hal_check_state_flags(&twai.context, TWAI_HAL_STATE_FLAG_BUS_OFF);
}

/// Helper function that will return true if the TWAI TX buffer is occupied.
static inline bool is_twai_tx_occupied()
{
    return twai_hal_check_state_flags(&twai.context, TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED);
}

/// Helper function that will purge the TWAI RX queue and wake the OpenMRN
/// stack if it was waiting for a frame to be ready to receive.
static inline void twai_purge_rx_queue()
{
    Notifiable* n = nullptr;
    {
        AtomicHolder h(&twai.buf_lock);
        LOG(VERBOSE, "ESP-TWAI: purging RX-Q:%zu", twai.rx_buf->pending());
        twai.stats.rx_missed += twai.rx_buf->pending();
        twai.rx_buf->flush();
        std::swap(n, twai.readable_notify);
    }
    if (n)
    {
        n->notify();
    }
#if CONFIG_VFS_SUPPORT_SELECT
    AtomicHolder l(&twai.select_lock);
    if (FD_ISSET(TWAI_VFS_FD, &twai.exceptfds_orig))
    {
        FD_SET(TWAI_VFS_FD, twai.exceptfds);
        esp_vfs_select_triggered(twai.select_sem);
    }
#endif // CONFIG_VFS_SUPPORT_SELECT
}

/// Helper function that will purge the TWAI TX queue and wake the OpenMRN
/// stack if it was waiting for TX space.
static inline void twai_purge_tx_queue()
{
    Notifiable* n = nullptr;
    {
        AtomicHolder h(&twai.buf_lock);
        LOG(VERBOSE, "ESP-TWAI: purging TX-Q:%zu", twai.tx_buf->pending());
        twai.stats.tx_lost += twai.tx_buf->pending();
        twai.tx_buf->flush();
        std::swap(n, twai.writable_notify);
    }
    if (n)
    {
        n->notify();
    }
#if CONFIG_VFS_SUPPORT_SELECT
    AtomicHolder l(&twai.select_lock);
    if (FD_ISSET(TWAI_VFS_FD, &twai.exceptfds_orig))
    {
        FD_SET(TWAI_VFS_FD, twai.exceptfds);
        esp_vfs_select_triggered(twai.select_sem);
    }
#endif // CONFIG_VFS_SUPPORT_SELECT
}

/// VFS adapter for write(fd, buf, size)
///
/// @param fd is the file descriptor being written to.
/// @param buf is the buffer containing the data to be written.
/// @param size is the size of the buffer.
/// @return number of bytes written or -1 if there is the write would be a
/// blocking operation.
static ssize_t twai_vfs_write(int fd, const void *buf, size_t size)
{
    LOG(VERBOSE, "ESP-TWAI: write(%d, %p, %zu)", fd, buf, size);
    DASSERT(fd == TWAI_VFS_FD);
    ssize_t sent = 0;
    const struct can_frame *data = (const struct can_frame *)buf;
    size /= sizeof(struct can_frame);
    bool bus_error = false;
    while (size && !bus_error)
    {
        if (is_twai_bus_off())
        {
            // If the TWAI bus is OFF initiate recovery and purge the pending TX queue.
            LOG_ERROR("ESP-TWAI: Bus is OFF, initiating recovery.");
            twai_hal_start_bus_recovery(&twai.context);
            bus_error = true;
            break;
        }
        else if (!is_twai_running())
        {
            LOG_ERROR("ESP-TWAI: TWAI driver is not running, unable to write "
                      "%zu frames.", size);
            bus_error = true;
            break;
        }
        else if (is_twai_err_passive())
        {
            // When the TWAI driver is in an error passive state it is not
            // possible to transmit additional frames, purge the TX queue and
            // track remaining frames as failed.
            //
            // NOTE: we are tracking pending remaining frames as written to
            // ensure the stack does not unnecessarily become blocked.

            sent += size;
            twai.stats.tx_lost += size;
            bus_error = true;
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
        else
        {
            twai.stats.tx_processed += frames_written;
        }

        if (is_twai_running() && !is_twai_tx_occupied() && frames_written)
        {
            // since the TX buffer is not occupied, retrieve the first
            // frame and transmit it here.
            AtomicHolder h(&twai.buf_lock);
            struct can_frame *frame = nullptr;
            twai_message_t tx_frame;
            twai_hal_frame_t hal_frame;
            if (twai.tx_buf->data_read_pointer(&frame) && frame != nullptr)
            {
                memset(&tx_frame, 0, sizeof(twai_message_t));
                tx_frame.identifier = frame->can_id;
                tx_frame.extd = IS_CAN_FRAME_EFF(*frame);
                tx_frame.rtr = IS_CAN_FRAME_RTR(*frame);
                tx_frame.data_length_code = frame->can_dlc;
                memcpy(tx_frame.data, frame->data, frame->can_dlc);
                twai_hal_format_frame(&tx_frame, &hal_frame);
                twai_hal_set_tx_buffer_and_transmit(&twai.context, &hal_frame);
            }
        }
        sent += frames_written;
        size -= frames_written;
    }

    if (bus_error)
    {
        twai_purge_tx_queue();
    }

    if (!sent)
    {
        errno = EWOULDBLOCK;
    }
    LOG(VERBOSE, "ESP-TWAI: write() %zu", sent * sizeof(struct can_frame));
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
    LOG(VERBOSE, "ESP-TWAI: read(%d, %p, %zu)", fd, buf, size);
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
        twai.stats.rx_processed += received_frames;
        size -= received_frames;
        received += received_frames;
        data += received_frames;
    }
    if (!received)
    {
        errno = EWOULDBLOCK;
        return -1;
    }

    LOG(VERBOSE, "ESP-TWAI: read() %zu", received * sizeof(struct can_frame));
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

    LOG(VERBOSE, "ESP-TWAI: Starting TWAI driver on:%s mode:%x (%s) fd:%d",
        path, mode, twai.non_blocking ? "non-blocking" : "blocking", 
        TWAI_VFS_FD);
    twai_purge_rx_queue();
    twai_purge_tx_queue();
    twai_hal_start(&twai.context, TWAI_MODE_NORMAL);
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
    LOG(INFO, "ESP-TWAI: Disabling TWAI driver using fd:%d", fd);
    twai_purge_rx_queue();
    twai_purge_tx_queue();
    twai_hal_stop(&twai.context);
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

#if CONFIG_VFS_SUPPORT_SELECT
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
    AtomicHolder l(&twai.select_lock);
    // zero the cached copy of the fd_sets before setting the incoming copy in
    // case the TWAI VFS FD is not set so we do not raise the alert when there
    // is an interesting event.
    FD_ZERO(&twai.readfds_orig);
    FD_ZERO(&twai.writefds_orig);
    FD_ZERO(&twai.exceptfds_orig);

    // If the TWAI FD is present in any of the FD sets we should process the
    // select call.
    if (FD_ISSET(TWAI_VFS_FD, readfds) || FD_ISSET(TWAI_VFS_FD, writefds) ||
        FD_ISSET(TWAI_VFS_FD, exceptfds))
    {
        twai.select_sem = sem;
        twai.readfds = readfds;
        twai.readfds_orig = *readfds;
        twai.writefds = writefds;
        twai.writefds_orig = *writefds;
        twai.exceptfds = exceptfds;
        twai.exceptfds_orig = *exceptfds;

        // zero the fd_sets so we can mark the correct signals when we trigger
        // the VFS layer.
        FD_ZERO(readfds);
        FD_ZERO(writefds);
        FD_ZERO(exceptfds);

        // Check if we have pending frames to RX, if so trigger an early exit
        // from select()
        if (FD_ISSET(TWAI_VFS_FD, &twai.readfds_orig))
        {
            AtomicHolder h(&twai.buf_lock);
            if (twai.rx_buf->pending())
            {
                FD_SET(TWAI_VFS_FD, readfds);
                esp_vfs_select_triggered(sem);
            }
        }
    }
    return ESP_OK;
}

/// VFS interface helper invoked when select() is woken up.
///
/// @param end_select_args is any arguments provided in vfs_start_select().
static esp_err_t twai_vfs_end_select(void *end_select_args)
{
    AtomicHolder l(&twai.select_lock);
    // zero the cached copy of the fd_sets to prevent triggering the VFS wakeup
    // since the select() has ended.
    FD_ZERO(&twai.readfds_orig);
    FD_ZERO(&twai.writefds_orig);
    FD_ZERO(&twai.exceptfds_orig);
    return ESP_OK;
}

#endif // CONFIG_VFS_SUPPORT_SELECT

/// TWAI Interrupt handler for receiving one (or more) TWAI frames.
static inline uint32_t twai_rx_frames()
{
    AtomicHolder h(&twai.buf_lock);
    uint32_t rx_ready_count = twai_hal_get_rx_msg_count(&twai.context);
    struct can_frame *can_frame = nullptr;
    uint32_t rx_count = 0;
    ESP_EARLY_LOGV(TWAI_LOG_TAG, "rx-ready-count: %" PRIu32, rx_ready_count);
    for (uint32_t idx = 0; idx < rx_ready_count; idx++)
    {
        twai_hal_frame_t frame;
        if (twai_hal_read_rx_buffer_and_clear(&twai.context, &frame))
        {
            if (frame.dlc > TWAI_FRAME_MAX_DLC)
            {
                // DLC is longer than supported, discard the frame.
                twai.stats.rx_discard++;
                ESP_EARLY_LOGE(TWAI_LOG_TAG, "rx-discard:%zu",
                               twai.stats.rx_discard);
            }
            else if (twai.rx_buf->data_write_pointer(&can_frame))
            {
                twai_message_t rx_frame;
                twai_hal_parse_frame(&frame, &rx_frame);
                memcpy(can_frame->data, rx_frame.data, TWAI_FRAME_MAX_DLC);
                can_frame->can_dlc = rx_frame.data_length_code;
                can_frame->can_id = rx_frame.identifier;
                if (rx_frame.extd)
                {
                    SET_CAN_FRAME_EFF(*can_frame);
                }
                else
                {
                    CLR_CAN_FRAME_EFF(*can_frame);
                }
                if (rx_frame.rtr)
                {
                    SET_CAN_FRAME_RTR(*can_frame);
                }
                else
                {
                    CLR_CAN_FRAME_RTR(*can_frame);
                }
                rx_count += twai.rx_buf->advance(1);
                ESP_EARLY_LOGV(TWAI_LOG_TAG, "rx-OK");
            }
            else
            {
                twai.stats.rx_missed++;
                ESP_EARLY_LOGV(TWAI_LOG_TAG, "rx-missed:%zu",
                               twai.stats.rx_missed);
            }
        }
        else
        {
            ESP_EARLY_LOGV(TWAI_LOG_TAG, "rx-overrun");
// If the SOC does not support automatic clearing of the RX FIFO we need to
// handle it here and break out of the loop.
#ifndef SOC_TWAI_SUPPORTS_RX_STATUS
            twai.stats.rx_overrun +=
                twai_hal_clear_rx_fifo_overrun(&twai.context);
            break;
#else
            twai.stats.rx_overrun++;
#endif // SOC_TWAI_SUPPORTS_RX_STATUS
        }
    }

    return rx_count;
}

/// TWAI Interrupt handler for sending a TWAI frame to the transmit buffer.
static inline uint32_t twai_tx_frame()
{
    AtomicHolder h(&twai.buf_lock);
    if (twai_hal_check_last_tx_successful(&twai.context))
    {
        ESP_EARLY_LOGV(TWAI_LOG_TAG, "TX-OK");
        twai.stats.tx_success++;
        twai.tx_buf->consume(1);
    }
    else
    {
        ESP_EARLY_LOGV(TWAI_LOG_TAG, "TX-FAIL");
        twai.stats.tx_failed++;
    }

    // Check if we have a pending frame to transmit in the queue
    struct can_frame *can_frame = nullptr;
    if (twai.tx_buf->data_read_pointer(&can_frame) && can_frame != nullptr)
    {
        twai_message_t tx_frame;
        twai_hal_frame_t hal_frame;
        memset(&tx_frame, 0, sizeof(twai_message_t));
        tx_frame.identifier = can_frame->can_id;
        tx_frame.extd = IS_CAN_FRAME_EFF(*can_frame);
        tx_frame.rtr =  IS_CAN_FRAME_RTR(*can_frame);
        tx_frame.data_length_code = can_frame->can_dlc;
        memcpy(tx_frame.data, can_frame->data, can_frame->can_dlc);
        twai_hal_format_frame(&tx_frame, &hal_frame);
        twai_hal_set_tx_buffer_and_transmit(&twai.context, &hal_frame);
        return 1;
    }
    return 0;
}

/// Interrupt handler for the TWAI device.
///
/// @param arg unused.
static void twai_isr(void *arg)
{
    BaseType_t wakeup = pdFALSE;
    uint32_t events = twai_hal_get_events(&twai.context);
    ESP_EARLY_LOGV(TWAI_LOG_TAG, "events:%04x", events);

#if defined(CONFIG_TWAI_ERRATA_FIX_RX_FRAME_INVALID) || \
    defined(CONFIG_TWAI_ERRATA_FIX_RX_FIFO_CORRUPT)
    if (events & TWAI_HAL_EVENT_NEED_PERIPH_RESET)
    {
        ESP_EARLY_LOGV(TWAI_LOG_TAG, "periph-reset");
        twai_hal_prepare_for_reset(&twai.context);
        periph_module_reset(PERIPH_TWAI_MODULE);
        twai_hal_recover_from_reset(&twai.context);
        twai.stats.rx_lost += twai_hal_get_reset_lost_rx_cnt(&twai.context);
#if CONFIG_VFS_SUPPORT_SELECT
        AtomicHolder l(&twai.select_lock);
        if (FD_ISSET(TWAI_VFS_FD, &twai.exceptfds_orig))
        {
            FD_SET(TWAI_VFS_FD, twai.exceptfds);
            esp_vfs_select_triggered_isr(twai.select_sem, &wakeup);
        }
#endif // CONFIG_VFS_SUPPORT_SELECT
    }
#endif // TWAI_ERRATA_FIX_RX_FRAME_INVALID || TWAI_ERRATA_FIX_RX_FIFO_CORRUPT

    // RX completed
    if ((events & TWAI_HAL_EVENT_RX_BUFF_FRAME) && twai_rx_frames())
    {
#if CONFIG_VFS_SUPPORT_SELECT
        AtomicHolder l(&twai.select_lock);
        if (FD_ISSET(TWAI_VFS_FD, &twai.readfds_orig))
        {
            FD_SET(TWAI_VFS_FD, twai.readfds);
            esp_vfs_select_triggered_isr(twai.select_sem, &wakeup);
        }
#endif // CONFIG_VFS_SUPPORT_SELECT
        // std::swap is not guaranteed to be in IRAM, so it is not used here.
        if (twai.readable_notify)
        {
            twai.readable_notify->notify_from_isr();
            twai.readable_notify = nullptr;
        }
    }

    // TX completed
    if ((events & TWAI_HAL_EVENT_TX_BUFF_FREE) && twai_tx_frame())
    {
#if CONFIG_VFS_SUPPORT_SELECT
        AtomicHolder l(&twai.select_lock);
        if (FD_ISSET(TWAI_VFS_FD, &twai.writefds_orig))
        {
            FD_SET(TWAI_VFS_FD, twai.writefds);
            esp_vfs_select_triggered_isr(twai.select_sem, &wakeup);
        }
#endif // CONFIG_VFS_SUPPORT_SELECT
        // std::swap is not guaranteed to be in IRAM, so it is not used here.
        if (twai.writable_notify)
        {
            twai.writable_notify->notify_from_isr();
            twai.writable_notify = nullptr;
        }
    }

    // Bus recovery complete, trigger a restart
    if (events & TWAI_HAL_EVENT_BUS_RECOV_CPLT)
    {
        ESP_EARLY_LOGV(TWAI_LOG_TAG, "bus recovery complete");
        // start the driver automatically
        twai_hal_start(&twai.context, TWAI_MODE_NORMAL);
    }

    // Bus error detected
    // NOTE: this will be raised even after entering error-passive state and
    // should be excluded from the bus_error increment accordingly.
    if (events & TWAI_HAL_EVENT_BUS_ERR && !is_twai_err_passive())
    {
        twai.stats.bus_error++;
        ESP_EARLY_LOGV(TWAI_LOG_TAG, "bus-error:%zu", twai.stats.bus_error);
    }

    // Arbitration error detected
    if (events & TWAI_HAL_EVENT_ARB_LOST)
    {
        twai.stats.arb_loss++;
        ESP_EARLY_LOGV(TWAI_LOG_TAG, "arb-loss:%zu", twai.stats.arb_loss);
    }

    if (wakeup == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

/// Background task used for periodic reporting of TWAI bus status and bus
/// watchdog.
///
/// The watchdog feature will handle the following use cases:
/// 1) TWAI Bus status remains in Recovering state for more than one reporting
/// interval. If this occurs the task will attempt to restart bus recovery.
/// 2) RX queue does not change and at least one frame is pending RX by the
/// OpenMRN stack. If this occurs the RX queue will be drained and an attempt
/// to wake the OpenMRN stack will be performed. All pending RX frames will be
/// counted as "missed" in the status reporting.
/// 3) TX queue does not change and at least one frame is pending TX by the
/// TWAI driver. If this occurs the TX queue will be drained and an attempt
/// to wake the OpenMRN stack will be performed. All pending TX frames will be
/// counted as "fail" in the status reporting.
///
/// These use cases can occur if the CAN bus has been disconnected or there is
/// a general failure in communicating with the CAN transceiver IC.
void* twai_watchdog(void* param)
{
    LOG(VERBOSE, "ESP-TWAI: Starting TWAI watchdog and reporting task");
    size_t last_rx_pending = 0;
    size_t last_tx_pending = 0;
    size_t last_tx_success = 0;
    uint32_t last_twai_state = 0;

    while (twai.active)
    {
        // delay until the next reporting interval, this is being used instead
        // of vTaskDelay to allow early wake up in the case of shutdown of the
        // TWAI driver.
        ulTaskNotifyTake(pdTRUE, STATUS_PRINT_INTERVAL);

        // If we wake up and the TWAI driver is no longer active we should exit
        // this loop for shutdown.
        if (!twai.active)
        {
            break;
        }

        // If the last status of the bus and current status are the same and it
        // is in a recovery state, retrigger the recovery as it will remain
        // stuck indefinitely without a retrigger.
        if (last_twai_state == twai.context.state_flags &&
            is_twai_recovering())
        {
            LOG(WARNING,
                "ESP-TWAI: Bus appears to be stuck, initiating bus recovery.");
            twai_hal_start_bus_recovery(&twai.context);
        }
        last_twai_state = twai.context.state_flags;

        // If the RX queue has not changed since our last check, purge the RX
        // queue and track it as missed frames.
        if (last_rx_pending && last_rx_pending == twai.rx_buf->pending())
        {
            LOG_ERROR("ESP-TWAI: RX-Q appears stuck, purging RX-Q!");
            twai_purge_rx_queue();
        }
        last_rx_pending = twai.rx_buf->pending();

        // If the TX queue has not changed since our last check and the TX
        // success count has not changed since our last check, purge the TX
        // queue and track it as failed frames.
        if (last_tx_pending && last_tx_pending == twai.tx_buf->pending() &&
            last_tx_success && last_tx_success == twai.stats.tx_success)
        {
            LOG_ERROR("ESP-TWAI: TX-Q appears stuck, purging TX-Q!");
            twai_purge_tx_queue();
        }
        last_tx_pending = twai.tx_buf->pending();
        last_tx_success = twai.stats.tx_success;

        if (twai.report_stats)
        {
            LOG(INFO,
                "ESP-TWAI: "
                "RX:%zu (pending:%zu,overrun:%zu,discard:%zu,miss:%zu,lost:%zu) "
                "TX:%zu (pending:%zu,suc:%zu,fail:%zu,lost:%zu) "
                "Bus (arb-loss:%zu,err:%zu,state:%s)",
                twai.stats.rx_processed, twai.rx_buf->pending(),
                twai.stats.rx_overrun, twai.stats.rx_discard,
                twai.stats.rx_missed, twai.stats.rx_lost,
                twai.stats.tx_processed, twai.tx_buf->pending(),
                twai.stats.tx_success, twai.stats.tx_failed,
                twai.stats.tx_lost, twai.stats.arb_loss, twai.stats.bus_error,
                is_twai_running() ? "Running" :
                is_twai_recovering() ? "Recovering" :
                is_twai_err_warn() ? "Err-Warn" :
                is_twai_err_passive() ? "Err-Pasv" :
                "Bus Off");
        }
    }
    LOG(VERBOSE, "ESP-TWAI: Stopping TWAI watchdog and reporting task");

    return NULL;
}

Esp32HardwareTwai::Esp32HardwareTwai(
    int rx, int tx, bool report, size_t rx_size, size_t tx_size,
    const char *path, int clock_out, int bus_status, uint32_t isr_core)
    : rxPin_(rx), txPin_(tx), extClockPin_(clock_out),
    busStatusPin_(bus_status), preferredIsrCore_(isr_core), vfsPath_(path)
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

    memset(&twai.stats, 0, sizeof(esp32_twai_stats_t));

    twai.rx_buf = DeviceBuffer<struct can_frame>::create(rx_size);
    HASSERT(twai.rx_buf != nullptr);

    twai.tx_buf =
        DeviceBuffer<struct can_frame>::create(tx_size, tx_size / 2);
    HASSERT(twai.tx_buf != nullptr);

    twai.report_stats = report;
}

Esp32HardwareTwai::~Esp32HardwareTwai()
{
    if (twai.active)
    {
        esp_intr_free(twai.isr_handle);
        twai_hal_deinit(&twai.context);
    }
    twai.active = false;

    esp_vfs_unregister(vfsPath_);

    twai.tx_buf->destroy();
    twai.rx_buf->destroy();

    if (twai.wd_thread)
    {
        xTaskNotifyGive(twai.wd_thread);
    }
}

/// Internal function which is used to allocate the TWAI ISR on a specific
/// core. This will be either directly called (for single core SoCs) in
/// Esp32HardwareTwai::hw_init() or via esp_ipc.
static void esp32_twai_isr_init(void *param)
{
    LOG(VERBOSE, "ESP-TWAI: Allocating ISR");
    ESP_ERROR_CHECK(
        esp_intr_alloc(ETS_TWAI_INTR_SOURCE, TWAI_INTERRUPT_FLAGS, twai_isr,
            nullptr, &twai.isr_handle));
}

void Esp32HardwareTwai::hw_init()
{
    LOG(VERBOSE,
        "ESP-TWAI: Configuring TWAI (TX:%d, RX:%d, EXT-CLK:%d, BUS-CTRL:%d)",
        txPin_, rxPin_, extClockPin_, busStatusPin_);
    gpio_set_pull_mode((gpio_num_t)txPin_, GPIO_FLOATING);
    esp_rom_gpio_connect_out_signal(txPin_, TWAI_TX_IDX, false, false);
    esp_rom_gpio_pad_select_gpio(txPin_);

    gpio_set_pull_mode((gpio_num_t)rxPin_, GPIO_FLOATING);
    esp_rom_gpio_connect_in_signal(rxPin_, TWAI_RX_IDX, false);
    esp_rom_gpio_pad_select_gpio(rxPin_);
    gpio_set_direction((gpio_num_t)rxPin_, GPIO_MODE_INPUT);

    if (extClockPin_ != GPIO_NUM_NC)
    {
        gpio_set_pull_mode((gpio_num_t)extClockPin_, GPIO_FLOATING);
        esp_rom_gpio_connect_out_signal(extClockPin_, TWAI_CLKOUT_IDX, false,
            false);
        esp_rom_gpio_pad_select_gpio((gpio_num_t)extClockPin_);
    }

    if (busStatusPin_ != GPIO_NUM_NC)
    {
        gpio_set_pull_mode((gpio_num_t)busStatusPin_, GPIO_FLOATING);
        esp_rom_gpio_connect_out_signal(extClockPin_, TWAI_BUS_OFF_ON_IDX,
            false, false);
        esp_rom_gpio_pad_select_gpio((gpio_num_t)busStatusPin_);
    }

    esp_vfs_t vfs = {};
    vfs.write = twai_vfs_write;
    vfs.read = twai_vfs_read;
    vfs.open = twai_vfs_open;
    vfs.close = twai_vfs_close;
    vfs.fcntl = twai_vfs_fcntl;
    vfs.ioctl = twai_vfs_ioctl;
#if CONFIG_VFS_SUPPORT_SELECT
    vfs.start_select = twai_vfs_start_select;
    vfs.end_select = twai_vfs_end_select;
#endif // CONFIG_VFS_SUPPORT_SELECT
    vfs.flags = ESP_VFS_FLAG_DEFAULT;
    ESP_ERROR_CHECK(esp_vfs_register(vfsPath_, &vfs, this));

    periph_module_reset(PERIPH_TWAI_MODULE);
    periph_module_enable(PERIPH_TWAI_MODULE);

    twai_timing_config_t timingCfg = TWAI_TIMING_CONFIG_125KBITS();
    twai_filter_config_t filterCfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,1,0)
    // default clock source if not specified in config.
    if (timingCfg.clk_src == 0)
    {
        timingCfg.clk_src = TWAI_CLK_SRC_DEFAULT;
    }
    twai_hal_config_t twai_hal_cfg = 
    {
        .controller_id = 0,
        .clock_source_hz = 0,
    };

    // retrieve the clock frequency from the SoC
    esp_clk_tree_src_get_freq_hz((soc_module_clk_t)timingCfg.clk_src,
        ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &twai_hal_cfg.clock_source_hz);

    // BRP validations
    uint32_t brp = timingCfg.brp;
    if (timingCfg.quanta_resolution_hz)
    {
        HASSERT(twai_hal_cfg.clock_source_hz % timingCfg.quanta_resolution_hz == 0);
        brp = twai_hal_cfg.clock_source_hz / timingCfg.quanta_resolution_hz;
    }
    HASSERT(twai_ll_check_brp_validation(brp));

    // Initialize the low level HAL APIs
    HASSERT(twai_hal_init(&twai.context, &twai_hal_cfg));
#else
    // Initialize the low level HAL APIs
    HASSERT(twai_hal_init(&twai.context));
#endif // IDF v5.1+

    LOG(VERBOSE, "ESP-TWAI: Initiailizing peripheral");
    twai_hal_configure(&twai.context, &timingCfg, &filterCfg,
        TWAI_DEFAULT_INTERRUPTS, 0);

#ifndef CONFIG_FREERTOS_UNICORE
    ESP_ERROR_CHECK(
        esp_ipc_call_blocking(preferredIsrCore_, esp32_twai_isr_init, nullptr));
#else
    esp32_twai_isr_init(nullptr);
#endif // CONFIG_FREERTOS_UNICORE
    twai.active = true;

    os_thread_create(&twai.wd_thread, "TWAI-WD", WATCHDOG_TASK_PRIORITY,
        WATCHDOG_TASK_STACK, twai_watchdog, this);
}

void Esp32HardwareTwai::get_driver_stats(esp32_twai_stats_t *stats)
{
    HASSERT(stats != nullptr);
    memcpy(stats, &twai.stats, sizeof(esp32_twai_stats_t));
}

} // namespace openmrn_arduino

#endif // ESP_PLATFORM
