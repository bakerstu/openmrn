
#include "Stm32Gpio.hxx"
#include "utils/GpioInitializer.hxx"
#include "BlinkerGPIO.hxx"

GPIO_PIN(LED_GREEN_RAW, LedPin, B, 3);

typedef GpioInitializer<LED_GREEN_RAW_Pin> GpioInit;

typedef LED_GREEN_RAW_Pin BLINKER_RAW_Pin;
typedef BLINKER_Pin LED_GREEN_Pin;
