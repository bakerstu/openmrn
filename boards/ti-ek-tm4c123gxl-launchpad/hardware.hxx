#ifndef _HARDWARE_HXX_
#define _HARDWARE_HXX_

#include "freertos_drivers/ti/TivaGPIO.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "driverlib/rom_map.h"
#include "utils/GpioInitializer.hxx"

#define TIVADCC_TIVA

GPIO_PIN(SW1, GpioInputPU, F, 4);
GPIO_PIN(SW2, GpioInputPU, F, 0);

GPIO_PIN(LED_RED, LedPin, F, 1);
GPIO_PIN(LED_GREEN, LedPin, F, 3);
GPIO_PIN(LED_BLUE_RAW, LedPin, F, 2);

typedef LED_BLUE_RAW_Pin BLINKER_RAW_Pin;
typedef BLINKER_Pin LED_BLUE_Pin;

GPIO_HWPIN(UART0RX, GpioHwPin, A, 0, U0RX, UART);
GPIO_HWPIN(UART0TX, GpioHwPin, A, 1, U0TX, UART);

GPIO_PIN(USB1, GpioUSBAPin, D, 4);
GPIO_PIN(USB2, GpioUSBAPin, D, 5);

GPIO_HWPIN(CAN0RX, GpioHwPin, E, 4, CAN0RX, CAN);
GPIO_HWPIN(CAN0TX, GpioHwPin, E, 5, CAN0TX, CAN);

GPIO_HWPIN(BOOSTER_H, GpioHwPin, B, 6, T0CCP0, Timer);
GPIO_HWPIN(BOOSTER_L, GpioHwPin, B, 7, T0CCP1, Timer);

GPIO_PIN(RAILCOM_TRIGGER, GpioOutputSafeHigh, D, 6);

GPIO_HWPIN(RAILCOM_CH1, GpioHwPin, B, 0, U1RX, UART);

typedef GpioInitializer<                          //
    SW1_Pin, SW2_Pin,                             //
    LED_RED_Pin, LED_GREEN_Pin, LED_BLUE_RAW_Pin, //
    USB1_Pin, USB2_Pin,                           //
    UART0RX_Pin, UART0TX_Pin,                     //
    BOOSTER_H_Pin, BOOSTER_L_Pin,                 //
    RAILCOM_TRIGGER_Pin, RAILCOM_CH1_Pin,         //
    CAN0RX_Pin, CAN0TX_Pin> GpioInit;

namespace TDebug {
using Resync = DummyPin;
using NextPacket = DummyPin;
};


#endif // _HARDWARE_HXX_
