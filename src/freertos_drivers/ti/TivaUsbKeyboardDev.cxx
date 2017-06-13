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
 * \file TivaUsbKeyboardDev.cxx
 * Device driver for a Tiva USB HID Keyboard device.
 *
 * @author Balazs Racz
 * @date 23 May 2017
 */

#include "freertos_drivers/ti/TivaUsbKeyboardDev.hxx"

//****************************************************************************
//
// The languages supported by this device.
//
//****************************************************************************
static const uint8_t g_pui8LangDescriptor[] = {
    4, USB_DTYPE_STRING, USBShort(USB_LANG_EN_US)};

//****************************************************************************
//
// The manufacturer string.
//
//****************************************************************************
static const uint8_t g_pui8ManufacturerString[] = {
    (17 + 1) * 2, USB_DTYPE_STRING, 'T', 0, 'e', 0, 'x', 0, 'a', 0, 's', 0, ' ',
    0, 'I', 0, 'n', 0, 's', 0, 't', 0, 'r', 0, 'u', 0, 'm', 0, 'e', 0, 'n', 0,
    't', 0, 's', 0,
};

//****************************************************************************
//
// The product string.
//
//****************************************************************************
static const uint8_t g_pui8ProductString[] = {(16 + 1) * 2, USB_DTYPE_STRING,
    'K', 0, 'e', 0, 'y', 0, 'b', 0, 'o', 0, 'a', 0, 'r', 0, 'd', 0, ' ', 0, 'E',
    0, 'x', 0, 'a', 0, 'm', 0, 'p', 0, 'l', 0, 'e', 0};

//****************************************************************************
//
// The serial number string.
//
//****************************************************************************
static const uint8_t g_pui8SeriailNumberString[] = {(8 + 1) * 2,
    USB_DTYPE_STRING, '1', 0, '2', 0, '3', 0, '4', 0, '5', 0, '6', 0, '7', 0,
    '8', 0};

//*****************************************************************************
//
// The interface description string.
//
//*****************************************************************************
static const uint8_t g_pui8HIDInterfaceString[] = {(22 + 1) * 2,
    USB_DTYPE_STRING, 'H', 0, 'I', 0, 'D', 0, ' ', 0, 'K', 0, 'e', 0, 'y', 0,
    'b', 0, 'o', 0, 'a', 0, 'r', 0, 'd', 0, ' ', 0, 'I', 0, 'n', 0, 't', 0, 'e',
    0, 'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0};

//*****************************************************************************
//
// The configuration description string.
//
//*****************************************************************************
static const uint8_t g_pui8ConfigString[] = {(26 + 1) * 2, USB_DTYPE_STRING,
    'H', 0, 'I', 0, 'D', 0, ' ', 0, 'K', 0, 'e', 0, 'y', 0, 'b', 0, 'o', 0, 'a',
    0, 'r', 0, 'd', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0, 'f', 0, 'i', 0, 'g', 0,
    'u', 0, 'r', 0, 'a', 0, 't', 0, 'i', 0, 'o', 0, 'n', 0};

//*****************************************************************************
//
// The descriptor string table.
//
//*****************************************************************************
static const uint8_t *const g_ppui8StringDescriptors[] = {g_pui8LangDescriptor,
    g_pui8ManufacturerString, g_pui8ProductString, g_pui8SeriailNumberString,
    g_pui8HIDInterfaceString, g_pui8ConfigString};

#define NUM_STRING_DESCRIPTORS                                                 \
    (sizeof(g_ppui8StringDescriptors) / sizeof(uint8_t *))

//*****************************************************************************
//
// A mapping from the ASCII value received from the UART to the corresponding
// USB HID usage code.
//
//*****************************************************************************
static const int8_t g_ppi8KeyUsageCodes[][2] = {
    {0, 0},                                          // 0x00
    {0, 0},                                          // 0x01
    {0, 0},                                          // 0x02
    {0, 0},                                          // 0x03
    {0, 0},                                          // 0x04
    {0, 0},                                          // 0x05
    {0, 0},                                          // 0x06
    {0, 0},                                          // 0x07
    {0, HID_KEYB_USAGE_BACKSPACE},                   // 0x08  '\b'
    {0, 0},                                          // 0x09
    {0, HID_KEYB_USAGE_ENTER},                       // 0x0a  '\n'
    {0, 0},                                          // 0x0b
    {0, 0},                                          // 0x0c
    {0, 0},                                          // 0x0d
    {0, 0},                                          // 0x0e
    {0, 0},                                          // 0x0f
    {0, 0},                                          // 0x10
    {0, 0},                                          // 0x11
    {0, 0},                                          // 0x12
    {0, 0},                                          // 0x13
    {0, 0},                                          // 0x14
    {0, 0},                                          // 0x15
    {0, 0},                                          // 0x16
    {0, 0},                                          // 0x17
    {0, 0},                                          // 0x18
    {0, 0},                                          // 0x19
    {0, 0},                                          // 0x1a
    {0, 0},                                          // 0x1b
    {0, 0},                                          // 0x1c
    {0, 0},                                          // 0x1d
    {0, 0},                                          // 0x1e
    {0, 0},                                          // 0x1f
    {0, HID_KEYB_USAGE_SPACE},                       //   0x20
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_1},         // ! 0x21
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_FQUOTE},    // " 0x22
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_3},         // # 0x23
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_4},         // $ 0x24
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_5},         // % 0x25
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_7},         // & 0x26
    {0, HID_KEYB_USAGE_FQUOTE},                      // ' 0x27
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_9},         // ( 0x28
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_0},         // ) 0x29
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_8},         // * 0x2a
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_EQUAL},     // + 0x2b
    {0, HID_KEYB_USAGE_COMMA},                       // , 0x2c
    {0, HID_KEYB_USAGE_MINUS},                       // - 0x2d
    {0, HID_KEYB_USAGE_PERIOD},                      // . 0x2e
    {0, HID_KEYB_USAGE_FSLASH},                      // / 0x2f
    {0, HID_KEYB_USAGE_0},                           // 0 0x30
    {0, HID_KEYB_USAGE_1},                           // 1 0x31
    {0, HID_KEYB_USAGE_2},                           // 2 0x32
    {0, HID_KEYB_USAGE_3},                           // 3 0x33
    {0, HID_KEYB_USAGE_4},                           // 4 0x34
    {0, HID_KEYB_USAGE_5},                           // 5 0x35
    {0, HID_KEYB_USAGE_6},                           // 6 0x36
    {0, HID_KEYB_USAGE_7},                           // 7 0x37
    {0, HID_KEYB_USAGE_8},                           // 8 0x38
    {0, HID_KEYB_USAGE_9},                           // 9 0x39
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_SEMICOLON}, // : 0x3a
    {0, HID_KEYB_USAGE_SEMICOLON},                   // ; 0x3b
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_COMMA},     // < 0x3c
    {0, HID_KEYB_USAGE_EQUAL},                       // = 0x3d
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_PERIOD},    // > 0x3e
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_FSLASH},    // ? 0x3f
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_2},         // @ 0x40
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_A},         // A 0x41
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_B},         // B 0x42
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_C},         // C 0x43
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_D},         // D 0x44
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_E},         // E 0x45
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_F},         // F 0x46
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_G},         // G 0x47
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_H},         // H 0x48
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_I},         // I 0x49
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_J},         // J 0x4a
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_K},         // K 0x4b
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_L},         // L 0x4c
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_M},         // M 0x4d
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_N},         // N 0x4e
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_O},         // O 0x4f
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_P},         // P 0x50
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_Q},         // Q 0x51
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_R},         // R 0x52
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_S},         // S 0x53
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_T},         // T 0x54
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_U},         // U 0x55
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_V},         // V 0x56
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_W},         // W 0x57
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_X},         // X 0x58
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_Y},         // Y 0x59
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_Z},         // Z 0x5a
    {0, HID_KEYB_USAGE_LBRACKET},                    // [ 0x5b
    {0, HID_KEYB_USAGE_BSLASH},                      // \ 0x5c
    {0, HID_KEYB_USAGE_RBRACKET},                    // ] 0x5d
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_6},         // ^ 0x5e
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_MINUS},     // _ 0x5f
    {0, HID_KEYB_USAGE_BQUOTE},                      // ` 0x60
    {0, HID_KEYB_USAGE_A},                           // a 0x61
    {0, HID_KEYB_USAGE_B},                           // b 0x62
    {0, HID_KEYB_USAGE_C},                           // c 0x63
    {0, HID_KEYB_USAGE_D},                           // d 0x64
    {0, HID_KEYB_USAGE_E},                           // e 0x65
    {0, HID_KEYB_USAGE_F},                           // f 0x66
    {0, HID_KEYB_USAGE_G},                           // g 0x67
    {0, HID_KEYB_USAGE_H},                           // h 0x68
    {0, HID_KEYB_USAGE_I},                           // i 0x69
    {0, HID_KEYB_USAGE_J},                           // j 0x6a
    {0, HID_KEYB_USAGE_K},                           // k 0x6b
    {0, HID_KEYB_USAGE_L},                           // l 0x6c
    {0, HID_KEYB_USAGE_M},                           // m 0x6d
    {0, HID_KEYB_USAGE_N},                           // n 0x6e
    {0, HID_KEYB_USAGE_O},                           // o 0x6f
    {0, HID_KEYB_USAGE_P},                           // p 0x70
    {0, HID_KEYB_USAGE_Q},                           // q 0x71
    {0, HID_KEYB_USAGE_R},                           // r 0x72
    {0, HID_KEYB_USAGE_S},                           // s 0x73
    {0, HID_KEYB_USAGE_T},                           // t 0x74
    {0, HID_KEYB_USAGE_U},                           // u 0x75
    {0, HID_KEYB_USAGE_V},                           // v 0x76
    {0, HID_KEYB_USAGE_W},                           // w 0x77
    {0, HID_KEYB_USAGE_X},                           // x 0x78
    {0, HID_KEYB_USAGE_Y},                           // y 0x79
    {0, HID_KEYB_USAGE_Z},                           // z 0x7a
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_LBRACKET},  // { 0x7b
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_BSLASH},    // | 0x7c
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_RBRACKET},  // } 0x7d
    {HID_KEYB_LEFT_SHIFT, HID_KEYB_USAGE_BQUOTE},    // ~ 0x7e
};

static uint32_t static_keyboard_handler(
    void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgData, void *pvMsgData)
{
    return TivaUsbKeyboardDev::instance()->keyboard_handler(
        ui32Event, ui32MsgData, pvMsgData);
}

TivaUsbKeyboardDev::TivaUsbKeyboardDev(const char *name, uint32_t interrupt)
    : Serial(name, 32, 1)
    , tivaKeyboardDevice_{USB_VID_TI_1CBE, USB_PID_KEYBOARD, 500,
          USB_CONF_ATTR_SELF_PWR | USB_CONF_ATTR_RWAKE, static_keyboard_handler,
          (void *)&tivaKeyboardDevice_, g_ppui8StringDescriptors,
          NUM_STRING_DESCRIPTORS}
    , isConnected_(0)
    , isSuspended_(0)
    , txPending_(0)
    , keyUpPending_(0)
{
    /* USB interrupt low priority.  We make this a low priority because
     * USB interrupts perform a decent amount of processing and we want
     * to maintain the overall determinism of our system.
     */
    MAP_IntPrioritySet(interrupt, 0xff);

    USBStackModeSet(0, eUSBModeDevice, 0);
    USBDHIDKeyboardInit(0, &tivaKeyboardDevice_);
}

void TivaUsbKeyboardDev::tx_char()
{
    if (!isConnected_)
    {
        return;
    }
    if (isSuspended_)
    {
        USBDHIDKeyboardRemoteWakeupRequest((void *)&tivaKeyboardDevice_);
        txPending_ = true;
        return;
    }
    if (!txPending_)
    {
        send_next_event();
    }
}

bool TivaUsbKeyboardDev::send_next_event()
{
    if (txBuf->pending() == 0)
    {
        return true;
    }
    HASSERT(!txPending_);
    txPending_ = true;
    uint8_t *data;
    txBuf->data_read_pointer(&data);
    uint32_t ui32Char = (*data);
    if (keyUpPending_)
    {
        if (USBDHIDKeyboardKeyStateChange((void *)&tivaKeyboardDevice_, 0,
                g_ppi8KeyUsageCodes[ui32Char][1], false) != KEYB_SUCCESS)
        {
            ++errorCount_;
        }
        keyUpPending_ = 0;
        txBuf->consume(1);
        return true;
    }
    else
    {
        if (USBDHIDKeyboardKeyStateChange((void *)&tivaKeyboardDevice_,
                g_ppi8KeyUsageCodes[ui32Char][0],
                g_ppi8KeyUsageCodes[ui32Char][1], true) != KEYB_SUCCESS)
        {
            ++errorCount_;
        }
        keyUpPending_ = 1;
        return false;
    }
}

uint32_t TivaUsbKeyboardDev::keyboard_handler(
    uint32_t ui32Event, uint32_t ui32MsgData, void *pvMsgData)
{
    switch (ui32Event)
    {
        //
        // The host has connected to us and configured the device.
        //
        case USB_EVENT_CONNECTED:
        {
            isConnected_ = true;
            isSuspended_ = false;
            send_next_event();
            break;
        }

        //
        // The host has disconnected from us.
        //
        case USB_EVENT_DISCONNECTED:
        {
            isConnected_ = false;
            txPending_ = false;
            break;
        }

        //
        // We receive this event every time the host acknowledges transmission
        // of a report.
        //
        case USB_EVENT_TX_COMPLETE:
        {
            txPending_ = false;
            if (send_next_event())
            {
                txBuf->signal_condition_from_isr();
            }
            break;
        }

        //
        // This event indicates that the host has suspended the USB bus.
        //
        case USB_EVENT_SUSPEND:
        {
            isSuspended_ = true;
            break;
        }

        //
        // This event signals that the host has resumed signalling on the bus.
        //
        case USB_EVENT_RESUME:
        {
            isSuspended_ = false;
            txPending_ = false;
            if (send_next_event())
            {
                txBuf->signal_condition_from_isr();
            }
            break;
        }

        //
        // This event indicates that the host has sent us an Output or
        // Feature report and that the report is now in the buffer we provided
        // on the previous USBD_HID_EVENT_GET_REPORT_BUFFER callback.
        //
        case USBD_HID_KEYB_EVENT_SET_LEDS:
        {
            uint8_t response = ui32MsgData & 0xff;
            if (!rxBuf->put(&response, 1))
            {
                overrunCount++;
            }
            else
            {
                rxBuf->signal_condition_from_isr();
            }
            break;
        }
    }
    return 0;
}

extern "C" {
/** Handle interrupts for USB0.
 */
void usb0_interrupt_handler(void)
{
    USB0DeviceIntHandler();
}

} // extern "C"
