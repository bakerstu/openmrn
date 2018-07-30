
#include "Stm32Gpio.hxx"
#include "utils/GpioInitializer.hxx"
#include "BlinkerGPIO.hxx"
#include "DummyGPIO.hxx"
#include "PWM.hxx"

GPIO_PIN(LED_GREEN_RAW, LedPin, A, 5);

GPIO_PIN(SW_USER, GpioInputPU, C, 13);

GPIO_PIN(SRV1, GpioOutputSafeHigh, C, 9);
GPIO_PIN(SRV2, GpioOutputSafeHigh, C, 7);
GPIO_PIN(SRV3, GpioOutputSafeHigh, C, 8);
GPIO_PIN(SRV4, GpioOutputSafeHigh, C, 6);
GPIO_PIN(SRV5, GpioOutputSafeHigh, F, 1);
GPIO_PIN(SRV6, GpioOutputSafeHigh, A, 15);
GPIO_PIN(SRV7, GpioOutputSafeHigh, C, 12);
GPIO_PIN(SRV8, GpioOutputSafeHigh, D, 2);

GPIO_PIN(TDRV1, GpioOutputSafeLow, B, 12);
GPIO_PIN(TDRV2, GpioOutputSafeLow, B, 2);
GPIO_PIN(TDRV3, GpioOutputSafeLow, B, 15);
GPIO_PIN(TDRV4, GpioOutputSafeLow, B, 1);
GPIO_PIN(TDRV5, GpioOutputSafeLow, A, 0);
GPIO_PIN(TDRV6, GpioOutputSafeLow, A, 4);
GPIO_PIN(TDRV7, GpioOutputSafeLow, C, 3);
GPIO_PIN(TDRV8, GpioOutputSafeLow, C, 0);

// Chip select line on the expansion port
GPIO_PIN(EXT_CS, GpioOutputSafeHigh, C, 10);
// Latch line assigned to the expansion port
GPIO_PIN(EXT_LAT, GpioOutputSafeLow, C, 11);
// Latch line on the onboard outputs
GPIO_PIN(OUT_LAT, GpioOutputSafeLow, A, 8);
// Latch line on the onboard inputs
GPIO_PIN(INP_LAT, GpioOutputSafeLow, A, 6);

typedef GpioInitializer<LED_GREEN_RAW_Pin, SW_USER_Pin, //
    SRV1_Pin, SRV2_Pin, SRV3_Pin, SRV4_Pin,             //
    SRV5_Pin, SRV6_Pin, SRV7_Pin, SRV8_Pin,             //
    TDRV1_Pin, TDRV2_Pin, TDRV3_Pin, TDRV4_Pin,         //
    TDRV5_Pin, TDRV6_Pin, TDRV7_Pin, TDRV8_Pin,         //
    EXT_CS_Pin, EXT_LAT_Pin, OUT_LAT_Pin, INP_LAT_Pin,  //
    DummyPin>
    GpioInit;

typedef LED_GREEN_RAW_Pin BLINKER_RAW_Pin;
typedef BLINKER_Pin LED_GREEN_Pin;

extern PWM* servo_channels[];
