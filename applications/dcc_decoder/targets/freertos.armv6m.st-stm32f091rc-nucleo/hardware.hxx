
#include "BlinkerGPIO.hxx"
#include "Stm32Gpio.hxx"
#include "utils/GpioInitializer.hxx"

GPIO_PIN(LED_GREEN_RAW, LedPin, A, 5);

GPIO_PIN(SW_USER, GpioInputPU, C, 13);

GPIO_PIN(DCC_IN, GpioInputPU, A, 6);
GPIO_PIN(RAILCOM_TX, GpioOutputSafeHigh, B, 6);

GPIO_PIN(DEBUG1, GpioOutputSafeLow, C, 9);
GPIO_PIN(DEBUG2, GpioOutputSafeLow, C, 8);

typedef GpioInitializer< //
    LED_GREEN_RAW_Pin,   //
    SW_USER_Pin,         //
    DCC_IN_Pin,          //
    DEBUG1_Pin,          //
    DEBUG2_Pin>
    GpioInit;

typedef LED_GREEN_RAW_Pin BLINKER_RAW_Pin;
typedef BLINKER_Pin LED_GREEN_Pin;
