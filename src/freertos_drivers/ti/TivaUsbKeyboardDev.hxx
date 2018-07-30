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
 * \file TivaUsbKeyboardDev.hxx
 * Device driver for a Tiva USB HID Keyboard device.
 *
 * @author Balazs Racz
 * @date 23 May 2017
 */

#ifndef _FREERTOS_DRIVERS_TI_TIVAUSBKEYBOARDDEV_HXX_
#define _FREERTOS_DRIVERS_TI_TIVAUSBKEYBOARDDEV_HXX_

#ifndef gcc
#define gcc
#endif

#include <stdint.h>

#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "usblib/usblib.h"
#include "usblib/usbhid.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdhid.h"
#include "usblib/device/usbdhidkeyb.h"
#include "usblib/usb-ids.h"

#include "freertos_drivers/common/Serial.hxx"
#include "utils/Singleton.hxx"

/// This device driver appears as a virtual serial port to the OpenMRN
/// application, and appears as an USB keyboard to the connected computer. Any
/// strings that are written to the serial device from OpenMRN will be sent as
/// keystrokes typed to the host computer.
class TivaUsbKeyboardDev : public Serial, public Singleton<TivaUsbKeyboardDev>
{
public:
    TivaUsbKeyboardDev(const char *name, uint32_t interrupt);

    void tx_char() override;

    void enable() override {}
    void disable() override {}

    /// Callback from the keyboard driver. Called in an interrupt context.
    uint32_t keyboard_handler(
        uint32_t ui32Event, uint32_t ui32MsgData, void *pvMsgData);

private:
    /// Internal implementation that sends a single keyboard event, from a
    /// critical section or in an interrupt context.
    /// @return true if the txBuf needs to be signaled.
    bool send_next_event();
    /// Number of transmission errors. These occur in the USB device stack.
    unsigned errorCount_{0};
    /// Keyboard driver structure
    tUSBDHIDKeyboardDevice tivaKeyboardDevice_;

    /// Whether the USB host has connected to this device.
    uint8_t isConnected_ : 1;
    /// Whether the USB bus is suspended. This means we need a special wakeup
    /// call before sending more keys.
    uint8_t isSuspended_ : 1;
    /// Whether we have a pending unfinished send.
    uint8_t txPending_ : 1;
    /// Whether we need to send a key up command still.
    uint8_t keyUpPending_ : 1;
};

#endif // _FREERTOS_DRIVERS_TI_TIVAUSBKEYBOARDDEV_HXX_
