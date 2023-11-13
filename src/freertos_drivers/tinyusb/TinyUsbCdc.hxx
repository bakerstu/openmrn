/** \copyright
 * Copyright (c) 2023, Balazs Racz
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
 * \file TinyUsbCdc.hxx
 * Base class for implementing a CDC Device driver using the TinyUsb stack.
 *
 * @author Balazs Racz
 * @date 13 Nov 2023
 */

#ifndef _FREERTOS_DRIVERS_TINYUSB_TINYUSBCDC_HXX_
#define _FREERTOS_DRIVERS_TINYUSB_TINYUSBCDC_HXX_

#include "Serial.hxx"
#include "os/OS.hxx"

class TinyUsbCdc : public Serial, public Singleton<TinyUsbCdc> {
public:
    TinyUsbCdc(const char* name)
        : Serial(name, 128 /* tx buffer size */, 0 /* no rx buffer */) {}

    // Call this function once from hw_postinit.
    void hw_postinit();
    
private:

    /** Function to try and transmit a character.
     */
    void tx_char() override;

    /** Device select method. Default impementation returns true.
     * @param file reference to the file
     * @param mode FREAD for read active, FWRITE for write active, 0 for
     *        exceptions
     * @return true if active, false if inactive
     */
    bool select(File* file, int mode) override;
    
    /** Read from a file or device.
     * @param file file reference for this device
     * @param buf location to place read data
     * @param count number of bytes to read
     * @return number of bytes read upon success, -1 upon failure with errno containing the cause
     */
    ssize_t read(File *file, void *buf, size_t count) override;

    /// Thread for running the tiny usb device stack.
    class UsbDeviceThread : public OSThread {
    public:
        UsbDeviceThread();
        
        void entry() override;
    } usbdThread_;
};




#endif // _FREERTOS_DRIVERS_TINYUSB_TINYUSBCDC_HXX_
