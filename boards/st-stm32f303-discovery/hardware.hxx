#ifndef _HARDWARE_HXX_
#define _HARDWARE_HXX_

#include "freertos_drivers/st/Stm32Gpio.hxx"
#include "utils/GpioInitializer.hxx"

GPIO_PIN(SW1, GpioInputPD, A, 0);

GPIO_PIN(LED_RAW_1, LedPin, E, 15);
GPIO_PIN(LED_2, LedPin, E, 14);
GPIO_PIN(LED_3, LedPin, E, 13);
GPIO_PIN(LED_4, LedPin, E, 12);
GPIO_PIN(LED_5, LedPin, E, 11);
GPIO_PIN(LED_6, LedPin, E, 10);
GPIO_PIN(LED_7, LedPin, E, 9);
GPIO_PIN(LED_8, LedPin, E, 8);

typedef LED_RAW_1_Pin BLINKER_RAW_Pin;

typedef GpioInitializer<                          //
    SW1_Pin,                                      //
    LED_RAW_1_Pin, LED_2_Pin, LED_3_Pin, LED_4_Pin,   //
    LED_5_Pin, LED_6_Pin, LED_7_Pin, LED_8_Pin>
    GpioInit;

#endif // _HARDWARE_HXX_
