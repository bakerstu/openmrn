#ifndef _HARDWARE_HXX_
#define _HARDWARE_HXX_

#include "freertos_drivers/ti/TivaGPIO.hxx"
#include "driverlib/rom_map.h"
#include "utils/GpioInitializer.hxx"

GPIO_PIN(LED_B1, LedPin, N, 1);
GPIO_PIN(LED_B2, LedPin, N, 0);
GPIO_PIN(LED_B3, LedPin, F, 4);
GPIO_PIN(LED_B4, LedPin, F, 0);

GPIO_PIN(SW1, GpioInputPU, J, 0);
GPIO_PIN(SW2, GpioInputPU, J, 1);

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

typedef GpioInitializer<                                            //
    SW1_Pin, SW2_Pin,                                               //
    LED_B1_Pin, LED_B2_Pin, LED_B3_Pin, LED_B4_Pin,                 //
    USB1_Pin, USB2_Pin,                                             //
    UART2RX_Pin, UART2TX_Pin,                                       //
    USB0EPEN_Pin, USB1_Pin, USB2_Pin, USB3_Pin, USB4_Pin, USB5_Pin, //
    CAN0RX_Pin, CAN0TX_Pin> GpioInit;

#endif // _HARDWARE_HXX_
