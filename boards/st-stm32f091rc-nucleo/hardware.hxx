
#include "freertos_drivers/st/Stm32Gpio.hxx"
#include "utils/GpioInitializer.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"

GPIO_PIN(LED_GREEN_RAW, LedPin, A, 5);

GPIO_PIN(SW_USER, GpioInputPU, C, 13);

typedef GpioInitializer<LED_GREEN_RAW_Pin,SW_USER_Pin> GpioInit;

typedef LED_GREEN_RAW_Pin BLINKER_RAW_Pin;
typedef BLINKER_Pin LED_GREEN_Pin;
