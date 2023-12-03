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
 * \file Stm32UsbCdc.hxx
 * Device driver for the STM32 devices using the TinyUsb stack.
 *
 * @author Balazs Racz
 * @date 13 Nov 2023
 */

#ifndef _FREERTOS_DRIVERS_ST_STM32CDCUSB_HXX_
#define _FREERTOS_DRIVERS_ST_STM32CDCUSB_HXX_

#include "freertos_drivers/tinyusb/TinyUsbCdc.hxx"

// Usage:
//
// Define an instance in HwInit.cxx.
// `Stm32UsbCdc serialUsb("/dev/serialusb");`
// in `hw_postinit()`, call `serialUsb.hw_postinit();`
//
// Make sure that the USB clock is set up in `hw_preinit()`. Use the Clock
// Recovery System if there is no crystal. Set up the USB pin map in
// hw_preinit.
class Stm32UsbCdc : public TinyUsbCdc
{
public:
    Stm32UsbCdc(const char *name)
        : TinyUsbCdc(name)
    {
    }
};

#endif // _FREERTOS_DRIVERS_ST_STM32CDCUSB_HXX_
