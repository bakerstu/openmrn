#ifndef _HARDWARE_HXX_
#define _HARDWARE_HXX_

#include "freertos_drivers/ti/TivaGPIO.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "driverlib/rom_map.h"
#include "utils/GpioInitializer.hxx"

typedef DummyPin BLINKER_RAW_Pin;

GPIO_HWPIN(UART0RX, GpioHwPin, A, 0, U0RX, UART);
GPIO_HWPIN(UART0TX, GpioHwPin, A, 1, U0TX, UART);

GPIO_PIN(USB1, GpioUSBAPin, D, 4);
GPIO_PIN(USB2, GpioUSBAPin, D, 5);

GPIO_HWPIN(CAN0RX, GpioHwPin, E, 4, CAN0RX, CAN);  //ok
GPIO_HWPIN(CAN0TX, GpioHwPin, E, 5, CAN0TX, CAN);  //ok

// Directly driven output pins. These go to JP8.
GPIO_PIN(O1, GpioOutputSafeLow, F, 0);
GPIO_PIN(O2, GpioOutputSafeLow, F, 1);
GPIO_PIN(O3, GpioOutputSafeLow, F, 2);
GPIO_PIN(O4, GpioOutputSafeLow, F, 3);
GPIO_PIN(O5, GpioOutputSafeLow, F, 4);
GPIO_PIN(O6, GpioOutputSafeLow, D, 0);
GPIO_PIN(O7, GpioOutputSafeLow, D, 1);
GPIO_PIN(O8, GpioOutputSafeLow, D, 6);

// Directly connected input pins. These come from JP5.
GPIO_PIN(I1, GpioInputPU, D, 2);
GPIO_PIN(I2, GpioInputPU, D, 3);
GPIO_PIN(I3, GpioInputPU, E, 0);
GPIO_PIN(I4, GpioInputPU, E, 1);
GPIO_PIN(I5, GpioInputPU, E, 2);
GPIO_PIN(I6, GpioInputPU, E, 3);
GPIO_PIN(I7, GpioInputPU, C, 6);
GPIO_PIN(I8, GpioInputPU, C, 7);

typedef GpioInitializer<            //
    O1_Pin, O2_Pin, O3_Pin, O4_Pin, //
    O5_Pin, O6_Pin, O7_Pin, O8_Pin, //
    I1_Pin, I2_Pin, I3_Pin, I4_Pin, //
    I5_Pin, I6_Pin, I7_Pin, I8_Pin, //
    USB1_Pin, USB2_Pin,             //
    UART0RX_Pin, UART0TX_Pin,       //
    CAN0RX_Pin, CAN0TX_Pin          //
    > GpioInit;

namespace TDebug {
using Resync = DummyPin;
using NextPacket = DummyPin;
};


#endif // _HARDWARE_HXX_
