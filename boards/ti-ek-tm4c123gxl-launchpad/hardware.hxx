#ifndef _HARDWARE_HXX_
#define _HARDWARE_HXX_

#include "TivaGPIO.hxx"
#include "driverlib/rom_map.h"
#include "utils/GpioInitializer.hxx"

GPIO_PIN(SW1, GpioInputPU, F, 4);
GPIO_PIN(SW2, GpioInputPU, F, 0);

GPIO_PIN(LED_RED_RAW, LedPin, F, 1);
GPIO_PIN(LED_GREEN, LedPin, F, 3);
GPIO_PIN(LED_BLUE, LedPin, F, 2);

GPIO_HWPIN(RAILCOM_CH1, GpioHwPin, B, 0, U1RX, UART);

GPIO_HWPIN(UART0RX, GpioHwPin, A, 0, U0RX, UART);
GPIO_HWPIN(UART0TX, GpioHwPin, A, 1, U0TX, UART);

GPIO_PIN(USB1, GpioUSBAPin, D, 4);
GPIO_PIN(USB2, GpioUSBAPin, D, 5);

GPIO_HWPIN(CAN0RX, GpioHwPin, E, 4, CAN0RX, CAN);
GPIO_HWPIN(CAN0TX, GpioHwPin, E, 5, CAN0TX, CAN);

typedef GpioInitializer<                          //
    SW1_Pin, SW2_Pin,                             //
    RAILCOM_CH1_Pin,                              //
    LED_RED_RAW_Pin, LED_GREEN_Pin, LED_BLUE_Pin, //
    USB1_Pin, USB2_Pin,                           //
    UART0RX_Pin, UART0TX_Pin,                     //
    CAN0RX_Pin, CAN0TX_Pin> GpioInit;

#endif // _HARDWARE_HXX_
