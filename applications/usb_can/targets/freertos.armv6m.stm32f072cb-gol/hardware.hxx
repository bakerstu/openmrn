
#include "Stm32Gpio.hxx"
#include "utils/GpioInitializer.hxx"

GPIO_PIN(LED_GREEN_RAW, LedPin, F, 0);
using LED_GREEN_Pin = ::InvertedGpio<LED_GREEN_RAW_Pin>;
GPIO_PIN(LED_OTHER_RAW, LedPin, F, 1);
using LED_OTHER_Pin = ::InvertedGpio<LED_OTHER_RAW_Pin>;

typedef GpioInitializer< //
    LED_GREEN_RAW_Pin, LED_OTHER_RAW_Pin>
    GpioInit;

typedef LED_GREEN_Pin BLINKER_RAW_Pin;
