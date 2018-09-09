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


#include "freertos_drivers/common/Serial.hxx"
#include "system_config.h"

extern "C" {
#include "driver/usb/usbfs/drv_usbfs.h"
#include "usb/usb_device.h"
#include "usb/usb_device_cdc.h"

extern const DRV_USBFS_INIT drvUSBFSInit;
extern const USB_DEVICE_INIT usbDevInitData;
} // extern "C"

#include "os/os.h"

/** Device driver for PIC32MX USB virtual serial port. */
class Pic32mxCdc : public Serial
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param interrupt interrupt number used by the device
     */
    Pic32mxCdc(const char *name);

    /** Destructor.
     */
    ~Pic32mxCdc()
    {
    }

    /** handle an interrupt.
     */
    void interrupt_handler()
    {
        DRV_USBFS_Tasks_ISR(drvUSBObject);
    }

    /** Background thread for doing USB device driver tasks. Must be called
     * aobut once per second in order to initialize, register with the USB
     * hosts, etc. */
    void device_tasks();

private:

    /** Function to try and transmit a character.  Unused by this device driver.
     */
    void tx_char() override {}

    void enable() override {} /**< function to enable device */
    void disable() override {} /**< function to disable device */

#if 0
    /** Write to a file or device.
     * @param file file reference for this device
     * @param buf location to find write data
     * @param count number of bytes to write
     * @return number of bytes written upon success, -1 upon failure with errno containing the cause
     */
    ssize_t write(File *file, const void *buf, size_t count) OVERRIDE;

    /** Device select method. Default impementation returns true.
     * @param file reference to the file
     * @param mode FREAD for read active, FWRITE for write active, 0 for
     *        exceptions
     * @return true if active, false if inactive
     */
    bool select(File* file, int mode) OVERRIDE;



    /** Handles CDC driver notifications related to control and setup of the
     * device.  This is called from within interrupt context.
     * @param data private data
     * @param event identifies the event we are being notified about
     * @param msg_param event-specific value
     * @param msg_data event-specific data
     * @return return value is event specific
     */
    static uint32_t control_callback(void *data, unsigned long event,
                                     unsigned long msg_param, void *msg_data);

    /** Handles CDC driver notifications related to reception.
     * This is called from within interrupt context.
     * @param data private data
     * @param event identifies the event we are being notified about
     * @param msg_param event-specific value
     * @param msg_data event-specific data
     * @return return value is event specific
     */
    static uint32_t rx_callback(void *data, unsigned long event,
                                unsigned long msg_param, void *msg_data);

    /** Handles CDC driver notifications related to transmission.
     * This is called from within interrupt context.
     * @param data private data
     * @param event identifies the event we are being notified about
     * @param msg_param event-specific value
     * @param msg_data event-specific data
     * @return return value is event specific
     */
    static uint32_t tx_callback(void *data, unsigned long event,
                                unsigned long msg_param, void *msg_data);

#endif

    //tUSBDCDCDevice usbdcdcDevice; /**< CDC serial device instance */
    SYS_MODULE_OBJ  drvUSBObject;
    SYS_MODULE_OBJ  usbDevObject0;
    uint32_t interrupt; /**< interrupt number for device */
    bool connected; /**< connection status */
    bool enabled; /**< enabled status */
    int woken; /**< task woken metadata for ISR */
    bool txPending; /**< true if a transmission is in progress or pending */
    SelectInfo selInfoWr; /**< Metadata for select() logic */

    /** Default constructor.
     */
    Pic32mxCdc();

    DISALLOW_COPY_AND_ASSIGN(Pic32mxCdc);
};

// Instantiates the driver.
Pic32mxCdc usbCdc0("/dev/serUSB0");

void* usb_device_task(void*) {
    while (1) {
        usbCdc0.device_tasks();
        vTaskDelay(configTICK_RATE_HZ / 20);
    }
}

extern "C" {
void usb_interrupt() {
    usbCdc0.interrupt_handler();
}
}

Pic32mxCdc::Pic32mxCdc(const char *name)
    : Serial(name)
{
    drvUSBObject = DRV_USBFS_Initialize(DRV_USBFS_INDEX_0, (SYS_MODULE_INIT *) &drvUSBFSInit);

    /* Set priority of USB interrupt source. This has to be within kernel
     * interrupt priority. */
    SYS_INT_VectorPrioritySet(INT_VECTOR_USB1, INT_PRIORITY_LEVEL2);

    /* Set Sub-priority of USB interrupt source */
    SYS_INT_VectorSubprioritySet(INT_VECTOR_USB1, INT_SUBPRIORITY_LEVEL0);

    /* Initialize Middleware */
    /* Initialize the USB device layer */
    usbDevObject0 = USB_DEVICE_Initialize (USB_DEVICE_INDEX_0 , ( SYS_MODULE_INIT* ) & usbDevInitData);

    os_thread_t tid;
    os_thread_create(&tid, "usb_tasks", 1, 512, &usb_device_task, nullptr);
}

void Pic32mxCdc::device_tasks() {
    DRV_USBFS_Tasks(drvUSBObject);
    USB_DEVICE_Tasks(usbDevObject0);
}
