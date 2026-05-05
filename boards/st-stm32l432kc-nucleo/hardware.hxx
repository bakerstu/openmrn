
#include "freertos_drivers/st/Stm32Gpio.hxx"
#include "utils/GpioInitializer.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"

GPIO_PIN(LED_GREEN_RAW, LedPin, B, 3);

GPIO_PIN(IN_A0, GpioInputPU, A, 0);
GPIO_PIN(IN_A1, GpioInputPU, A, 1);
GPIO_PIN(IN_A2, GpioInputPU, A, 3);
GPIO_PIN(IN_A3, GpioInputPU, A, 4);

GPIO_PIN(OUT_D3, GpioOutputSafeLow, B, 0);
GPIO_PIN(OUT_D4, GpioOutputSafeLow, B, 7);
GPIO_PIN(OUT_D5, GpioOutputSafeLow, B, 6);
GPIO_PIN(OUT_D6, GpioOutputSafeLow, B, 1);

typedef GpioInitializer<LED_GREEN_RAW_Pin,      //
    IN_A0_Pin, IN_A1_Pin, IN_A2_Pin, IN_A3_Pin, //
    OUT_D3_Pin, OUT_D4_Pin, OUT_D5_Pin, OUT_D6_Pin>
    GpioInit;

typedef LED_GREEN_RAW_Pin BLINKER_RAW_Pin;
typedef BLINKER_Pin LED_GREEN_Pin;
