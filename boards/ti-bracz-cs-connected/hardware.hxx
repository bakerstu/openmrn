

#ifndef _BRACZ_CS_TIVA_HARDWARE_HXX_
#define _BRACZ_CS_TIVA_HARDWARE_HXX_

#include "TivaGPIO.hxx"
#include "DummyGPIO.hxx"
#include "utils/GpioInitializer.hxx"

/*
#define LED_RED GPIO_PORTN_BASE, GPIO_PIN_4
#define LED_YELLOW GPIO_PORTQ_BASE, GPIO_PIN_0
#define LED_GREEN GPIO_PORTP_BASE, GPIO_PIN_4
#define LED_BLUE GPIO_PORTD_BASE, GPIO_PIN_2 */

GPIO_PIN(LED_RED, LedPin, N, 4);
GPIO_PIN(LED_YELLOW, LedPin, Q, 0);
GPIO_PIN(LED_GREEN, LedPin, P, 4);
GPIO_PIN(LED_BLUE, LedPin, D, 2);

/*
#define LED_B1 GPIO_PORTN_BASE, GPIO_PIN_1
#define LED_B2 GPIO_PORTN_BASE, GPIO_PIN_0
#define LED_B3 GPIO_PORTF_BASE, GPIO_PIN_4
#define LED_B4 GPIO_PORTF_BASE, GPIO_PIN_0
*/

GPIO_PIN(LED_B1, LedPin, N, 1);
GPIO_PIN(LED_B2, LedPin, N, 0);
GPIO_PIN(LED_B3, LedPin, F, 4);
GPIO_PIN(LED_B4, LedPin, F, 0);

GPIO_PIN(USR_SW1, GpioInputPU, J, 0);
GPIO_PIN(USR_SW2, GpioInputPU, J, 1);

GPIO_PIN(RC_DEBUG, GpioOutputSafeLow, K, 7);

namespace io
{
typedef LED_RED_Pin TrackPktLed;
typedef LED_BLUE_Pin TrackOnLed;
typedef LED_B1_Pin BlinkerLed;

typedef LED_YELLOW_Pin AccPwrLed;
typedef LED_B3_Pin GoPausedLed;
};

GPIO_HWPIN(UART2RX, GpioHwPin, D, 4, U2RX, UART);
GPIO_HWPIN(UART2TX, GpioHwPin, D, 5, U2TX, UART);

GPIO_HWPIN(CAN0RX, GpioHwPin, A, 0, CAN0RX, CAN);
GPIO_HWPIN(CAN0TX, GpioHwPin, A, 1, CAN0TX, CAN);

GPIO_HWPIN(USB0EPEN, GpioHwPin, D, 6, USB0EPEN, USBDigital);
GPIO_PIN(USB1, GpioUSBAPin, B, 0);
GPIO_PIN(USB2, GpioUSBAPin, B, 1);
GPIO_PIN(USB3, GpioUSBAPin, L, 6);
GPIO_PIN(USB4, GpioUSBAPin, L, 7);
GPIO_PIN(USB5, GpioInputNP, Q, 4);

GPIO_PIN(TDEBUG1, GpioOutputSafeLow, P, 5);
GPIO_PIN(TDEBUG2, GpioOutputSafeLow, M, 7);

typedef GpioInitializer<                                             //
    USR_SW1_Pin, USR_SW2_Pin,                                        //
    LED_B1_Pin, LED_B2_Pin, LED_B3_Pin, LED_B4_Pin,                  //
    USB1_Pin, USB2_Pin,                                              //
    UART2RX_Pin, UART2TX_Pin,                                        //
    TDEBUG1_Pin, TDEBUG2_Pin,                                        //
    USB0EPEN_Pin, USB1_Pin, USB2_Pin, USB3_Pin, USB4_Pin, USB5_Pin,  //
    CAN0RX_Pin, CAN0TX_Pin>
    GpioInit;

namespace TDebug {
using Resync = TDEBUG1_Pin;
using NextPacket = TDEBUG2_Pin;
//using Resync = DummyPin;
//using NextPacket = DummyPin;
};

#endif // _BRACZ_CS_TIVA_HARDWARE_HXX_
