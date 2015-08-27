
#include "Stm32Gpio.hxx"
#include "utils/GpioInitializer.hxx"


GPIO_PIN(LED3_RAW, LedPin, C, 8);
GPIO_PIN(LED4, LedPin, C, 9);
GPIO_PIN(LED5, LedPin, C, 6);
GPIO_PIN(LED6, LedPin, C, 7);


typedef GpioInitializer<                          //
    LED3_RAW_Pin, LED4_Pin, LED5_Pin, LED6_Pin> GpioInit;

typedef LED3_RAW_Pin BLINKER_RAW_Pin;
