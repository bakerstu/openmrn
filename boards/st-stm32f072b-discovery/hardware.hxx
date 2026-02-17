
#include "freertos_drivers/st/Stm32Gpio.hxx"
#include "utils/GpioInitializer.hxx"

GPIO_PIN(LED_ORA, LedPin, C, 8);
GPIO_PIN(LED_GREEN, LedPin, C, 9);
GPIO_PIN(LED_RED_RAW, LedPin, C, 6);
GPIO_PIN(LED_BLUE, LedPin, C, 7);

GPIO_PIN(SW_USER, GpioInputPD, A, 0);

typedef GpioInitializer<                          //
    LED_RED_RAW_Pin, LED_GREEN_Pin, LED_ORA_Pin, LED_BLUE_Pin> GpioInit;

typedef LED_RED_RAW_Pin BLINKER_RAW_Pin;
