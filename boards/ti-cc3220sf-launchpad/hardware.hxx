#ifndef _HARDWARE_HXX_
#define _HARDWARE_HXX_

#include "freertos_drivers/ti/CC3200GPIO.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "driverlib/rom_map.h"
#include "utils/GpioInitializer.hxx"

GPIO_PIN(SW2, GpioInputPin, A2, 6);
GPIO_PIN(SW3, GpioInputPin, A1, 5);

GPIO_PIN(LED_RED_RAW, GpioOutputSafeLow, A1, 1);
GPIO_PIN(LED_GREEN, GpioOutputSafeLow, A1, 3);
GPIO_PIN(LED_YELLOW, GpioOutputSafeLow, A1, 2);

typedef LED_RED_RAW_Pin BLINKER_RAW_Pin;
typedef BLINKER_Pin LED_RED_Pin;

// Create an initializer that can initialize all the GPIO pins in one shot
typedef GpioInitializer<SW2_Pin,
                        SW3_Pin,
                        LED_RED_RAW_Pin,
                        LED_GREEN_Pin,
                        LED_YELLOW_Pin> GpioInit;

#endif // _HARDWARE_HXX_
