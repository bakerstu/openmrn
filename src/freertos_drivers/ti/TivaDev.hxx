/** \copyright
 * Copyright (c) 2014, Stuart W Baker
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
 * \file TivaDev.hxx
 * This file implements the device class prototypes for TivaWare.
 *
 * @author Stuart W. Baker
 * @date 6 May 2014
 */

#ifndef _FREERTOS_DRIVERS_TI_TIVADEV_HXX_
#define _FREERTOS_DRIVERS_TI_TIVADEV_HXX_

#ifndef gcc
#define gcc
#endif

#include <cstdint>

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"

#include "Serial.hxx"
#include "Can.hxx"
#include "I2C.hxx"

/** Private data for this implementation of serial.
 */
class TivaCdc : public Serial
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param interrupt interrupt number used by the device
     */
    TivaCdc(const char *name, uint32_t interrupt);

    /** Destructor.
     */
    ~TivaCdc()
    {
    }

    /** @todo (Stuart Baker) this should be made private */
    /** handle an interrupt.
     */
    void interrupt_handler();

private:
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

    void enable() override; /**< function to enable device */
    void disable() override; /**< function to disable device */

    /** Function to try and transmit a character.  Unused by this device driver.
     */
    void tx_char() OVERRIDE {}

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

    tUSBDCDCDevice usbdcdcDevice; /**< CDC serial device instance */
    uint32_t interrupt; /**< interrupt number for device */
    bool connected; /**< connection status */
    bool enabled; /**< enabled status */
    int woken; /**< task woken metadata for ISR */
    bool txPending; /**< true if a transmission is in progress or pending */
    tLineCoding lineCoding; /**< line encoding for the device */
    SelectInfo selInfoWr; /**< Metadata for select() logic */

    /** Default constructor.
     */
    TivaCdc();

    DISALLOW_COPY_AND_ASSIGN(TivaCdc);
};

/** Specialization of Serial driver for Tiva UART.
 */
class TivaUart : public Serial
{
public:
    /** These mode bits need to be OR-ed together for the mode argument and
     * ioctl. */
    enum Mode
    {
        CS5 = UART_CONFIG_WLEN_5, /**< 5-bits word length */
        CS6 = UART_CONFIG_WLEN_6, /**< 6-bits word length */
        CS7 = UART_CONFIG_WLEN_7, /**< 7-bits word length */
        CS8 = UART_CONFIG_WLEN_8, /**< 8-bits word length */
        CSTOPB = UART_CONFIG_STOP_TWO, /**< send two stop bits instead of 1 */
    };

    /** Function point for the tx enable assert and deassert methods */
    typedef void (*TxEnableMethod)();
    
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param base base address of this device
     * @param interrupt interrupt number of this device
     * @param baud desired baud rate
     * @param mode to configure the UART for
     * @param hw_fifo true if hardware fifo is to be enabled, else false.
     * @param tx_enable_assert callback to assert the transmit enable
     * @param tx_enable_deassert callback to deassert the transmit enable
     */
    TivaUart(const char *name, unsigned long base, uint32_t interrupt,
        uint32_t baud = 115200, uint32_t mode = CS8, bool hw_fifo = true,
        TxEnableMethod tx_enable_assert = nullptr,
        TxEnableMethod tx_enable_deassert = nullptr);

    /** Destructor.
     */
    ~TivaUart()
    {
    }

    /** @todo (Stuart Baker) this should be made private */
    /** handle an interrupt.
     */
    void interrupt_handler();

    /** Request an ioctl transaction. Supported ioctl is TCSBRK, TCDRAINNOTIFY,
     * TCSTOP*, TCBAUDRATE and TCPAR* from include/freertos/tc_ioctl.h */
    int ioctl(File *file, unsigned long int key, unsigned long data) override;
    
private:
    void enable() override; /**< function to enable device */
    void disable() override; /**< function to disable device */

    /** Try and transmit a message.
     */
    void tx_char() override;

    /** Send data until there is no more space left.
     */
    void send();

    /** Sets the port baud rate and mode from the class variables. */
    void set_mode();

    /** function pointer to a method that asserts the transmit enable. */
    TxEnableMethod txEnableAssert_;

    /** function pointer to a method that deasserts the transmit enable. */
    TxEnableMethod txEnableDeassert_;
    
    /** Notifiable to invoke when the transmit engine has finished operation. */
    Notifiable* txComplete_{nullptr};
    
    uint32_t base_; /**< base address of this device */
    uint32_t interrupt_ : 8; /**< interrupt of this device */
    uint32_t baud_ : 24; /**< desired baud rate */
    uint8_t hwFIFO_; /**< enable HW FIFO */
    uint8_t uartMode_; /**< uart config (mode) flags */
    uint8_t txPending_; /**< transmission currently pending */
    uint8_t nineBit_; /**< true if using 9-bit reception */

    /** Default constructor.
     */
    TivaUart();

    DISALLOW_COPY_AND_ASSIGN(TivaUart);
};

/** Specialization of CAN driver for Tiva CAN.
 */
class TivaCan : public Can
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param base base address of this device
     * @param interrupt interrupt number of this device
     */
    TivaCan(const char *name, unsigned long base, uint32_t interrupt);

    /** Destructor.
     */
    ~TivaCan()
    {
    }

    /** @todo (Stuart Baker) this should be made private */
    /** handle an interrupt.
     */
    void interrupt_handler();

    /// Request an ioctl transaction.
    /// @param file file reference for this device
    /// @param key ioctl key
    /// @param data key data
    /// @return >= 0 upon success, -errno upon failure
    int ioctl(File *file, unsigned long int key, unsigned long data) override;

private:
    void enable() override; /**< function to enable device */
    void disable() override; /**< function to disable device */
    void tx_msg() override; /**< function to try and transmit a message */

    unsigned long base; /**< base address of this device */
    unsigned long interrupt; /**< interrupt of this device */
    bool txPending; /**< transmission currently pending */
    uint8_t canState; /**< current state of the CAN-bus. */
    /** Default constructor.
     */
    TivaCan();

    DISALLOW_COPY_AND_ASSIGN(TivaCan);
};

/** Specialization of Serial driver for Tiva UART.
 */
class TivaI2C : public I2C
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param base base address of this device
     * @param interrupt interrupt number of this device
     * @param fast_mode true for 400kHz, false for 100kHz operation
     */
    TivaI2C(const char *name, unsigned long base, uint32_t interrupt,
        bool fast_mode = false);

    /** Destructor.
     */
    ~TivaI2C()
    {
    }

    /** @todo (Stuart Baker) this should be made private */
    /** handle an interrupt.
     */
    void interrupt_handler();

private:
    void enable() OVERRIDE {} /**< function to enable device */
    void disable() OVERRIDE {} /**< function to disable device */

    /** Method to transmit/receive the data.
     * @param msg message to transact.
     * @param stop produce a stop condition at the end of the transfer
     * @return bytes transfered upon success or -1 with errno set
     */
    int transfer(struct i2c_msg *msg, bool stop) OVERRIDE;

    unsigned long base; /**< base address of this device */
    unsigned long interrupt; /**< interrupt of this device */
    struct i2c_msg *msg_; /**< message for current transaction */
    bool stop_; /**< current transaction ends in a stop if true */
    int count_; /**< current count index within transaction */

    /** Semaphore to wakeup task level from ISR */
    OSSem sem;

    /** Default constructor.
     */
    TivaI2C();

    DISALLOW_COPY_AND_ASSIGN(TivaI2C);
};

#endif /* _FREERTOS_DRIVERS_TI_TIVADEV_HXX_ */
