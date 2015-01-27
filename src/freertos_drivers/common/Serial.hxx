/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file Serial.hxx
 * This file implements a generic serial device driver layer.
 *
 * @author Stuart W. Baker
 * @date 3 January 2013
 */

#ifndef _FREERTOS_DRIVERS_COMMON_SERIAL_HXX_
#define _FREERTOS_DRIVERS_COMMON_SERIAL_HXX_

#include "BlockOrWakeUp.hxx"
#include "SimpleLog.hxx"
#include "Devtab.hxx"
#include "nmranet_config.h"
#include "os/OS.hxx"

/** Private data for a serial device */
class Serial : public Node
{
protected:
    /** Constructor
     * @param name device name in file system
     */
    Serial(const char *name)
        : Node(name)
        , txQ(os_mq_create(config_serial_tx_buffer_size(),
                           sizeof(unsigned char)))
        , rxQ(os_mq_create(config_serial_rx_buffer_size(),
                           sizeof(unsigned char)))
        , overrunCount(0)
        , selInfoRd()
        , selInfoWr()
    {
    }    

    /** Destructor.
     */
    ~Serial()
    {
        /** @todo (Stuart Baker) for completeness we should destroy the
         * txQ and rxQ here.
         */
        HASSERT(0);
    }
    
    virtual void tx_char() = 0; /**< function to try and transmit a character */

    os_mq_t txQ; /**< transmit queue */
    os_mq_t rxQ; /**< receive queue */
    unsigned int overrunCount; /**< overrun count */

    SelectInfo selInfoRd; /**< select wakeup metadata for read active */
    SelectInfo selInfoWr; /**< select wakeup metadata for write active */

private:
    /** Read from a file or device.
     * @param file file reference for this device
     * @param buf location to place read data
     * @param count number of bytes to read
     * @return number of bytes read upon success, -1 upon failure with errno containing the cause
     */
    ssize_t read(File *file, void *buf, size_t count) OVERRIDE;

    /** Write to a file or device.
     * @param file file reference for this device
     * @param buf location to find write data
     * @param count number of bytes to write
     * @return number of bytes written upon success, -1 upon failure with errno containing the cause
     */
    ssize_t write(File *file, const void *buf, size_t count) OVERRIDE;

    /** Request an ioctl transaction
     * @param file file reference for this device
     * @param node node reference for this device
     * @param key ioctl key
     * @param data key data
     */
    int ioctl(File *file, unsigned long int key, unsigned long data) OVERRIDE;
    
    /** Device select method. Default impementation returns true.
     * @param file reference to the file
     * @param mode FREAD for read active, FWRITE for write active, 0 for
     *        exceptions
     * @return true if active, false if inactive
     */
    bool select(File* file, int mode) OVERRIDE;

    /** Discards all pending buffers. Called after disable(). */
    void flush_buffers() OVERRIDE;

    DISALLOW_COPY_AND_ASSIGN(Serial);
};

/* This is fixed and equals the USB packet size that the CDC device will
 * advertise to be able to receive. This is a performance parameter, 64 is the
 * largest packet size permitted by USB for virtual serial ports. */
#define USB_SERIAL_PACKET_SIZE 64

class USBSerialNode : public Node
{
public:
    USBSerialNode(const char *name)
        : Node(name)
        , txQEnd_(0)
        , rxQBegin_(0)
        , rxQEnd_(0)
        , txBlock_(Atomic())
        , rxBlock_(Atomic())
        , selInfoRd()
        , selInfoWr()
    {
    }

    ssize_t read(File *file, void *buf, size_t count) OVERRIDE;
    ssize_t write(File *file, const void *buf, size_t count) OVERRIDE;
    int ioctl(File *file, unsigned long int key, unsigned long data) OVERRIDE;
    void flush_buffers() OVERRIDE;

protected:
    /** Tells the driver that the tx of the previous packet has finished. The
     * driver may call back tx_from_isr inline if there is more data to send. */
    void tx_finished_from_isr();

    /** Signals from an ISR context that a regular context thread should be
     * woken up and call into the rx_packet_irqlocked function. */
    void set_rx_pending_from_isr()
    {
        /** TODO(balazs.racz): we should actually wake up someone here. */
        rxPending_ = 1;
    }

    /** Checks if the RX packet buffer is empty from an ISR context. If it is
     * empty, a non-null value is returned. The caller should then read the
     * packet into this buffer and call set_rx_finished_from_isr(size). */
    void* try_read_packet_from_isr();

    /** Notifies the driver that the RX buffer was filled from an IRQ
     * context. This shall only be called after try_read_packet_from_isr was
     * called. Will wake up an RX thread.
     * @param size is the number of bytes that were filled into the rx
     * buffer.*/
    void set_rx_finished_from_isr(uint8_t size);

    void tx_discard_packet_locked()
    {
        txPending_ = 0;
        // TODO: notify
    }

    /** Returns true if a write() called now would have some space to put data
     * into. */
    bool has_tx_buffer_free()
    {
        return txQEnd_ < USB_SERIAL_PACKET_SIZE;
    }

    /** Adds a byte to the tx buffer. */
    void add_to_tx_buffer(uint8_t data)
    {
        HASSERT(has_tx_buffer_free());
        txQ_[txQEnd_++] = data;
    }

    /** Marks that the tx buffer has been handed over to the hardware. */
    void mark_tx_buffer_sent()
    {
        txQEnd_ = 0;
    }

private:
    /** Device select method. Default impementation returns true.
     * @param file reference to the file
     * @param mode FREAD for read active, FWRITE for write active, 0 for
     *        exceptions
     * @return true if active, false if inactive
     */
    bool select(File* file, int mode) OVERRIDE;

    /** Requests a packet to be sent from an ISR context. The buffer will be
     * invalidated as soon as the call returns. Returns true if the packet was
     * sent, false if there was an error and the packet should be discarded. */
    virtual bool tx_packet_from_isr(const void* data, size_t len) = 0;

    /** Requests a packet to be sent from a regular context (but under a lock
     * and TX interrupt disabled).  The buffer will be invalidated and
     * overwritten as soon as the call returns. Returns true if the packet was
     * sent, false if it should be discarded or re-tried.  */
    virtual bool tx_packet_irqlocked(const void* data, size_t len) = 0;

    /** Tries to receive a packet (from a regular context, locked and RX
     * interrupt disabled). This is only ever called if the driver does
     * set_rx_pending_from_isr().
     * @returns the number of bytes read into the buffer. */
    virtual size_t rx_packet_irqlocked(void *data)
    {
        DIE("Must implement rx_packet_irqlocked if you use "
            "set_rx_pending_from_isr");
    }

    volatile uint8_t txQEnd_, rxQBegin_, rxQEnd_;
    volatile uint8_t txPending_ : 1;
    volatile uint8_t rxPending_ : 1;
    uint8_t txQ_[USB_SERIAL_PACKET_SIZE];
    uint8_t rxQ_[USB_SERIAL_PACKET_SIZE * 3];

    BlockOrWakeUp<Atomic> txBlock_;
    BlockOrWakeUp<Atomic> rxBlock_;

    SelectInfo selInfoRd; /**< select wakeup metadata for read active */
    SelectInfo selInfoWr; /**< select wakeup metadata for write active */

protected:
    LogBuffer log_;
};


#endif /* _FREERTOS_DRIVERS_COMMON_SERIAL_HXX_ */
