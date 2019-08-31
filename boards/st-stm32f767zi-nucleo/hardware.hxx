
#include "Stm32Gpio.hxx"
#include "utils/GpioInitializer.hxx"
#include "BlinkerGPIO.hxx"

GPIO_PIN(LED1_RAW, LedPin, B, 0);
GPIO_PIN(LED2, LedPin, B, 7);
GPIO_PIN(LED3, LedPin, B, 14);

GPIO_PIN(SW_USER, GpioInputPD, C, 13);

typedef GpioInitializer<LED1_RAW_Pin, LED2_Pin, LED3_Pin, SW_USER_Pin> GpioInit;

typedef LED1_RAW_Pin BLINKER_RAW_Pin;
typedef BLINKER_Pin LED1_Pin;
