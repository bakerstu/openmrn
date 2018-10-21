/** \copyright
 * Copyright (c) 2018, Balazss Racz
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
 * \file Pic32mxUsbCdcDevice.cxx
 *
 * This file implements a USB CDC device driver layer using PIC32 Harmony USB
 * middleware driver.
 *
 * @author Balazs Racz
 * @date 9 Sep 2018
 */

#include <fcntl.h>

#include "freertos_drivers/common/Serial.hxx"
#include "system_config.h"
#include "utils/Atomic.hxx"
#include "freertos_drivers/pic32mx/int_pic32mx795/int_defines.h"

extern "C"
{
#include "driver/usb/usbfs/drv_usbfs.h"
#include "usb/usb_device.h"
#include "usb/usb_device_cdc.h"

/// Initialization structure in flash
extern const DRV_USBFS_INIT drvUSBFSInit;
/// Initialization structure in flash
extern const USB_DEVICE_INIT usbDevInitData;
} // extern "C"

#include "os/os.h"

/// Device driver for PIC32MX USB virtual serial port.
///
/// High-level decisions:
///
/// The serial port driver is compiled without RTOS support. This means all the
/// mutexes it has internally are just bits. This can cause error values
/// returned if we have multiple calls from different thread + interrupt
/// context.
///
/// Major decision: whether we are calling CDC send/receive functions from the
/// interrupt handler or not. If we don't, then we need to ensure that a user
/// thread gets woken up when a transmit is complete and there are pending bytes
/// in the transmit buffer. That's somewhat difficult. It's easier to call the
/// next transmit inline in the interrupt handler, and also gives better
/// performance (fewer context switches, less CPU usage, more throughput).
///
/// In order to achieve this, we need to ensure that all calls to the USB stack
/// are protected by critical sections that prevent the interrupt from occuring.
///
/// A specialty of this USB driver is that write and read commands have
/// zero-copy API, which means that the buffer pointer we give to the stack
/// needs to remain available so long as the transfer is outstanding. We cannot
/// use the userspace buffer pointers and have O_NONBLOCK file descriptors. So
/// we are making one copy from user buffer into the device buffer, then give
/// that pointer to the USB stack.
///
/// For transmit buffer we are using the standard txBuf coming from ::Serial. We
/// set this transmit buffer to 3 urb length. This allows us to keep sending
/// 64-byte packets when the application stack keeps pushing bytes at the full
/// available throughput. For receive buffer there is a driver restriction to
/// always give a 64-byte buffer for read. We cannot do that with a single
/// DeviceBuffer (even if we set it to 128 byte long), thus we manage our own
/// receive buffers. We still use DeviceBuffer class, but two of them with
/// double buffering, as this combination allows us tracking the holes between
/// the two buffers when a short read comes in.
class Pic32mxCdc : public Serial, private Atomic
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     */
    Pic32mxCdc(const char *name);

    /// Destructor.
    ~Pic32mxCdc();

    /// Handle an interrupt.
    inline void interrupt_handler()
    {
        DRV_USBFS_Tasks_ISR(drvUSBObject);
    }

    /// Background thread for doing USB device driver tasks. Must be called a
    /// ~dozen times per second in order to initialize, register with the USB
    /// hosts, etc.
    inline void device_tasks();

    /// This function is called from the interrupt context of the USB driver to
    /// notify us about various events at the driver layer, such as USB plugged
    /// in.
    inline void event_hander_from_isr(USB_DEVICE_EVENT event, void *pData);

    /// This function is called from the interrupt context of the USB driver to
    /// notify us about various events at the CDC layer, such as TX complete.
    inline USB_DEVICE_CDC_EVENT_RESPONSE cdc_event_hander_from_isr(
        USB_DEVICE_CDC_EVENT event, void *pData);

private:
    static constexpr unsigned USB_CDC_BUFFER_SIZE = 64;

    static constexpr unsigned TX_BUFFER_SIZE = 3 * USB_CDC_BUFFER_SIZE;
    static constexpr unsigned RX_DEVICEBUFFER_SIZE = 0;

    /// Unused.
    void tx_char() override
    {
    }
    /// Called by the serial driver when the first fd gets opened.
    void enable() override;
    /// Called by the serial driver when the last fd gets closed.
    void disable() override;
    /// Called by the serial driver after the last fd gets closed.
    void flush_buffers() override;

    /** Read from a file or device.
     * @param file file reference for this device
     * @param buf location to place read data
     * @param count number of bytes to read
     * @return number of bytes read upon success, -1 upon failure with errno
     * containing the cause
     */
    ssize_t read(File *file, void *buf, size_t count) override;

    /** Write to a file or device.
     * @param file file reference for this device
     * @param buf location to find write data
     * @param count number of bytes to write
     * @return number of bytes written upon success, -1 upon failure with errno
     * containing the cause
     */
    ssize_t write(File *file, const void *buf, size_t count) OVERRIDE;

    /** Device select method. Default impementation returns true.
     * @param file reference to the file
     * @param mode FREAD for read active, FWRITE for write active, 0 for
     *        exceptions
     * @return true if active, false if inactive
     */
    bool select(File *file, int mode) override;

    /// Issues a read to the USB driver stack. Must be called from within a lock
    /// or critical section. Uses next_rx_buffer().
    void start_read_atomic();

    /// Issues a write to the USB driver stack. Must be called from within a
    /// lock or critical section or interrupt.
    void start_write_atomic();

    /// @return the side of the double buffer from which the userspace should be
    /// reading.
    DeviceBuffer<uint8_t> *current_rx_buffer()
    {
        return rxBuffers[currentReadBuffer];
    }

    /// @return the side of the double buffer into which the driver should be
    /// writing.
    DeviceBuffer<uint8_t> *next_rx_buffer()
    {
        return rxBuffers[currentReadBuffer ^ 1];
    }

    /// Module pointer for USB core device.
    SYS_MODULE_OBJ drvUSBObject;
    /// Module pointer for USB middleware device.
    SYS_MODULE_OBJ usbDevObject0;
    /// Equivalent of an fd for the USB core driver.
    USB_DEVICE_HANDLE deviceHandle {USB_DEVICE_HANDLE_INVALID};
    /// Holds last set line coding data that came from the host.
    USB_CDC_LINE_CODING lineCodingData;
    /// Handle for the current pending read.
    USB_DEVICE_CDC_TRANSFER_HANDLE readHandle;
    /// Handle for the current pending write.
    USB_DEVICE_CDC_TRANSFER_HANDLE writeHandle;

    // ==== These variables are protected by Atomic *this. ====

    /// True when we have open fds.
    unsigned isEnabled : 1;
    /// true wehen we need to execute an attach command on the usb task.
    unsigned needAttach : 1;
    /// true when we have a configured host.
    unsigned isConfigured : 1;
    /// Index of the buffer for current (userspace) reads. This buffer is
    /// considered full, even if it has zero bytes remaining. The steady state,
    /// when all input bytes are already consumed by the application layer, is
    /// that rxBuffer[currentReadBuffer] has zero bytes pending, rxPending is
    /// true, and rxBuffers[currentReadBuffer ^ 1] has its pointer submitted to
    /// the USB driver stack to receive the next urb from the host, whenever it
    /// comes.
    unsigned currentReadBuffer : 1;
    /// true when we have an outstanding read request with the USB driver.
    unsigned rxPending : 1;
    /// true if the opposite read buffer (the one after current) also has
    /// data. If this is true, then rxPending cannot be true.
    unsigned nextReadBufferFull : 1;
    /// Zero if no writes are pending, otherwise the number of bytes we sent
    /// with the pending write to the USB device driver.
    unsigned txPendingBytes : 7;

    /// Counts internal error conditions that should not occur but we are hoping
    /// we can recover from them.
    unsigned driverErrors : 10;

    /// unused. Do not remove -- needed to force linking in the interrupt
    /// trampoline.
    unsigned irqVector : 5;

    /// Receive buffers. We have these separate from ::Serial, because we need
    /// to use double buffering with the USB middleware; specifically we always
    /// need to provide a free 64-byte buffer for the read command.
    DeviceBuffer<uint8_t> *rxBuffers[2];

    /** Default constructor.
     */
    Pic32mxCdc();

    DISALLOW_COPY_AND_ASSIGN(Pic32mxCdc);
};

// Instantiates the driver.
Pic32mxCdc usbCdc0("/dev/serUSB0");

void *usb_device_task(void *)
{
    while (1)
    {
        usbCdc0.device_tasks();
        vTaskDelay(configTICK_RATE_HZ / 20);
    }
}

extern "C"
{
    void usb_interrupt()
    {
        usbCdc0.interrupt_handler();
    }
}

Pic32mxCdc::Pic32mxCdc(const char *name)
    : Serial(name, TX_BUFFER_SIZE, RX_DEVICEBUFFER_SIZE)
    , isEnabled(0)
    , needAttach(0)
    , isConfigured(0)
    , currentReadBuffer(0)
    , rxPending(0)
    , nextReadBufferFull(0)
    , txPendingBytes(0)
    , driverErrors(0)
    , irqVector(usb_interrupt_vector_number)
{
    // Allocates receive buffers.
    rxBuffers[0] = DeviceBuffer<uint8_t>::create(USB_CDC_BUFFER_SIZE);
    rxBuffers[1] = DeviceBuffer<uint8_t>::create(USB_CDC_BUFFER_SIZE);

    // Initialize Middleware
    drvUSBObject = DRV_USBFS_Initialize(
        DRV_USBFS_INDEX_0, (SYS_MODULE_INIT *)&drvUSBFSInit);

    // Set priority of USB interrupt source. This has to be within kernel
    // interrupt priority.
    SYS_INT_VectorPrioritySet(INT_VECTOR_USB1, INT_PRIORITY_LEVEL1);

    // Set Sub-priority of USB interrupt source
    SYS_INT_VectorSubprioritySet(INT_VECTOR_USB1, INT_SUBPRIORITY_LEVEL0);

    // Initialize the USB device layer
    usbDevObject0 = USB_DEVICE_Initialize(
        USB_DEVICE_INDEX_0, (SYS_MODULE_INIT *)&usbDevInitData);

    // Background thread to execute connection tasks.
    os_thread_t tid;
    os_thread_create(&tid, "usb_tasks", 1, 512, &usb_device_task, nullptr);

    // Setup default line coding parameters. The actual values are ignored, but
    // they can be queried and set by the host.
    lineCodingData.dwDTERate = 115200;
    lineCodingData.bDataBits = 8;
    lineCodingData.bParityType = 0;
    lineCodingData.bCharFormat = 0;
}

Pic32mxCdc::~Pic32mxCdc()
{
    rxBuffers[0]->destroy();
    rxBuffers[1]->destroy();
}

void Pic32mxCdc::enable()
{
    AtomicHolder h(this);
    isEnabled = true;
    // Throws away all pending data.
    rxBuffers[currentReadBuffer]->flush();
    if (nextReadBufferFull)
    {
        rxBuffers[1 - currentReadBuffer]->flush();
        nextReadBufferFull = 0;
    }
    if (!isConfigured)
    {
        return;
    }
    // Decides if we need to start a read.
    if (!rxPending)
    {
        start_read_atomic();
    }
}
void Pic32mxCdc::disable()
{
    AtomicHolder h(this);
    isEnabled = false;
}

void Pic32mxCdc::flush_buffers()
{
    AtomicHolder h(this);
    rxBuffers[0]->flush();
    rxBuffers[1]->flush();
    nextReadBufferFull = false;
    // Don't touch currentReadBuffer as there might be a pending RX. If the
    // pending receive completes, we'll throw away the data when the next ::open
    // happens.

    // We don't throw away the tx buf because the application may have sent data
    // with ::write but there is no way to wait for them for the tx buf to
    // clear.
}

void Pic32mxCdc::start_read_atomic()
{
    HASSERT(!rxPending);
    HASSERT(!nextReadBufferFull);
    rxPending = 1;
    auto *buf = next_rx_buffer();
    DASSERT(!buf->pending());
    buf->flush();
    uint8_t *data;
    volatile unsigned rem = buf->data_write_pointer(&data);
    HASSERT(rem >= USB_CDC_BUFFER_SIZE);
    auto ret = USB_DEVICE_CDC_Read(
        USB_DEVICE_CDC_INDEX_0, &readHandle, data, USB_CDC_BUFFER_SIZE);
    if (ret != USB_DEVICE_CDC_RESULT_OK)
    {
        ++driverErrors;
        rxPending = false;
    }
}

ssize_t Pic32mxCdc::read(File *file, void *buf, size_t count)
{
    uint8_t *data = static_cast<uint8_t *>(buf);
    ssize_t result = 0;

    while (count)
    {
        size_t bytes_read;
        {
            AtomicHolder h(this);
            if ((current_rx_buffer()->pending() == 0) && nextReadBufferFull)
            {
                // Swap read buffers.
                currentReadBuffer ^= 1;
                nextReadBufferFull = false;
            }
            // Starts new read request if needed.
            if (isConfigured && !rxPending && !nextReadBufferFull)
            {
                start_read_atomic();
            }
            bytes_read =
                current_rx_buffer()->get(data, count < 64 ? count : 64);
        }

        if (bytes_read == 0)
        {
            /* no more data to receive */
            if ((file->flags & O_NONBLOCK) || result > 0)
            {
                break;
            }
            else
            {
                // wait for data to come in. This calls select internally.
                rxBuffers[0]->block_until_condition(file, true);
            }
        }

        count -= bytes_read;
        result += bytes_read;
        data += bytes_read;
    }
    if (!result && (file->flags & O_NONBLOCK))
    {
        return -EAGAIN;
    }

    return result;
}

void Pic32mxCdc::start_write_atomic()
{
    HASSERT(!txPendingBytes);
    uint8_t *data;
    // We use data_read_pointer to get a pointer to a consecutive block of
    // bytes. This will automatically align us to 64-byte boundary every time
    // the ring buffer rolls over. The result is that if the application
    // suddenly starts sending a lot of data we'll end up batching everything
    // into full urbs.
    size_t len = txBuf->data_read_pointer(&data);
    if (!len)
        return;
    USB_DEVICE_CDC_TRANSFER_FLAGS flag =
        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE;
    if (len > USB_CDC_BUFFER_SIZE ||
        ((len == USB_CDC_BUFFER_SIZE) &&
            (txBuf->pending() > USB_CDC_BUFFER_SIZE)))
    {
        len = USB_CDC_BUFFER_SIZE;
        flag = USB_DEVICE_CDC_TRANSFER_FLAGS_MORE_DATA_PENDING;
    }
    txPendingBytes = len;
    auto ret = USB_DEVICE_CDC_Write(
        USB_DEVICE_CDC_INDEX_0, &writeHandle, data, len, flag);
    if (ret != USB_DEVICE_CDC_RESULT_OK)
    {
        ++driverErrors;
        txPendingBytes = 0;
    }
}

ssize_t Pic32mxCdc::write(File *file, const void *buf, size_t count)
{
    const unsigned char *data = (const unsigned char *)buf;
    ssize_t result = 0;

    while (count)
    {
        size_t bytes_written = 0;
        if (isConfigured)
        {
            AtomicHolder h(this);
            // We limit the amount of bytes we write with each iteration in
            // order to limit the amount of time that interrupts are disabled
            // and preserve our real-time performance.
            bytes_written = txBuf->put(data, count < 64 ? count : 64);
            if (txPendingBytes == 0)
            {
                start_write_atomic();
            }
        }

        if (bytes_written == 0)
        {
            /* no more data to receive */
            if ((file->flags & O_NONBLOCK) || result > 0)
            {
                break;
            }
            else
            {
                // wait for space to be available. will call select inside.
                txBuf->block_until_condition(file, false);
            }
        }
        else
        {
            count -= bytes_written;
            result += bytes_written;
            data += bytes_written;
        }
    }

    if (!result && (file->flags & O_NONBLOCK))
    {
        return -EAGAIN;
    }

    return result;
}

bool Pic32mxCdc::select(File *file, int mode)
{
    bool retval = false;

    AtomicHolder l(this);
    switch (mode)
    {
        case FREAD:
            // This complicated expression ensures that we wake up a reader
            // thread not only when there are actual bytes to read, but also
            // when we need to swap reader buffers or when we need to start the
            // initial read after usb is plugged into the host.
            if ((current_rx_buffer()->pending() > 0) || nextReadBufferFull ||
                (isConfigured && !nextReadBufferFull && !rxPending))
            {
                retval = true;
            }
            else
            {
                rxBuffers[0]->select_insert();
            }
            break;
        case FWRITE:
            // We ensure to wake up a writer thread when the usb just got
            // plugged in.
            if (isConfigured && ((txBuf->space() > 0) || (txPendingBytes == 0)))
            {
                retval = true;
            }
            else
            {
                txBuf->select_insert();
            }
            break;
        default:
        case 0:
            /* we don't support any exceptions */
            break;
    }

    return retval;
}

static void usb_device_event_handler(
    USB_DEVICE_EVENT event, void *pData, uintptr_t context)
{
    usbCdc0.event_hander_from_isr(event, pData);
}

void Pic32mxCdc::device_tasks()
{
    DRV_USBFS_Tasks(drvUSBObject);
    USB_DEVICE_Tasks(usbDevObject0);

    do
    {
        if (deviceHandle != USB_DEVICE_HANDLE_INVALID)
        {
            break;
        }
        deviceHandle =
            USB_DEVICE_Open(USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE);
        if (deviceHandle == USB_DEVICE_HANDLE_INVALID)
        {
            break;
        }
        USB_DEVICE_EventHandlerSet(deviceHandle, &usb_device_event_handler, 0);
        needAttach = 1;
    } while (0);

    if (needAttach)
    {
        USB_DEVICE_Attach(deviceHandle);
        needAttach = 0;
    }
}

static USB_DEVICE_CDC_EVENT_RESPONSE cdc_device_event_handler(
    USB_DEVICE_CDC_INDEX index, USB_DEVICE_CDC_EVENT event, void *pData,
    uintptr_t userData)
{
    return usbCdc0.cdc_event_hander_from_isr(event, pData);
}

void Pic32mxCdc::event_hander_from_isr(USB_DEVICE_EVENT event, void *pData)
{
    switch (event)
    {
        case USB_DEVICE_EVENT_POWER_REMOVED:
            USB_DEVICE_Detach(deviceHandle);
            break;
        case USB_DEVICE_EVENT_POWER_DETECTED:
            needAttach = 1;
            break;
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:
            isConfigured = 0;
            // Throws away all data pending to be written to the host.
            txBuf->flush();
            break;
        case USB_DEVICE_EVENT_CONFIGURED:
        {
            auto v =
                ((USB_DEVICE_EVENT_DATA_CONFIGURED *)pData)->configurationValue;
            if (v != 1)
            {
                break;
            }

            // Registers the CDC Device application event handler.
            USB_DEVICE_CDC_EventHandlerSet(0, &cdc_device_event_handler, 0);
            isConfigured = 1;
            // Wakes up readers and writers.
            rxBuffers[0]->signal_condition_from_isr();
            txBuf->signal_condition_from_isr();
            break;
        }
        case USB_DEVICE_EVENT_SUSPENDED:
            break;
        case USB_DEVICE_EVENT_RESUMED:
            break;

        default:
            break;
    }
}

USB_DEVICE_CDC_EVENT_RESPONSE Pic32mxCdc::cdc_event_hander_from_isr(
    USB_DEVICE_CDC_EVENT event, void *pData)
{
    switch (event)
    {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:
            // This means the host wants to know the current line coding. This
            // is a control transfer request.

            USB_DEVICE_ControlSend(
                deviceHandle, &lineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:
            // This means the host wants to set the line coding.  This is a
            // control transfer request.

            USB_DEVICE_ControlReceive(
                deviceHandle, &lineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:
            // Note: here we could implement fake hardware flow control by
            // looking at the DTR and DCD lines as given by the host. I don't
            // think that's interesting as USB drivers always have builtin
            // hardware flow control using kernel buffers.

            // fall through
        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:
            // acknowledge
            USB_DEVICE_ControlStatus(
                deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;
        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:
            break;
        case USB_DEVICE_CDC_EVENT_SEND_BREAK:
            break;
        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:
        {
            auto *params =
                static_cast<USB_DEVICE_CDC_EVENT_DATA_READ_COMPLETE *>(pData);
            next_rx_buffer()->advance(params->length);
            rxPending = false;
            nextReadBufferFull = true;
            rxBuffers[0]->signal_condition_from_isr();
            break;
        }
        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:
        {
            auto *params =
                static_cast<USB_DEVICE_CDC_EVENT_DATA_WRITE_COMPLETE *>(pData);
            DASSERT(params->length == txPendingBytes);
            txBuf->consume(params->length);
            txPendingBytes = 0;
            if (txBuf->pending())
            {
                start_write_atomic();
            }
            txBuf->signal_condition_from_isr();
            break;
        }
        default:
            break;
    }
}
