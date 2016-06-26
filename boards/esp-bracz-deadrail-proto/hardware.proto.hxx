#ifndef _ESP_BRACZ_DEADRAIL_PROTO_HARDWARE_HXX_
#define _ESP_BRACZ_DEADRAIL_PROTO_HARDWARE_HXX_

#include "freertos_drivers/esp8266/Esp8266Gpio.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"
#include "utils/GpioInitializer.hxx"

struct HW
{
    GPIO_PIN(BLINKER_RAW, GpioOutputSafeHigh, 2);
    static constexpr bool blinker_invert = true;

    GPIO_PIN(MOT_A_HI, GpioOutputSafeLow, 14);
    GPIO_PIN(MOT_A_LO, GpioOutputSafeLow, 12);

    GPIO_PIN(MOT_B_HI, GpioOutputSafeLow, 5);
    GPIO_PIN(MOT_B_LO, GpioOutputSafeLow, 13);

    static constexpr bool invertLow = false;

    // forward: A=HI B=LO

    // typedef BLINKER_Pin LIGHT_FRONT_Pin;
    typedef DummyPin LIGHT_FRONT_Pin;
    //GPIO_PIN(LIGHT_FRONT, GpioOutputSafeLow, ??? RX pin);
    typedef DummyPin LIGHT_BACK_Pin;
    //GPIO_PIN(LIGHT_BACK, GpioOutputSafeLow, 4);

    // Doubles as manual request pin.
    GPIO_PIN(REQ_BLOAD, GpioInputPU, 4);

    GPIO_PIN(ASEL1, GpioOutputSafeHigh, 0);
    typedef BLINKER_Pin ASEL2_Pin;
    //GPIO_PIN(ASEL2, GpioOutputSafeHigh, 2);

    GPIO_PIN(CHRG_EN, GpioOutputSafeLow, 15);

    typedef DummyPin BootloaderActivePin;

    typedef GpioInitializer<        //
        MOT_A_HI_Pin, MOT_A_LO_Pin, //
        MOT_B_HI_Pin, MOT_B_LO_Pin, //
        LIGHT_FRONT_Pin, LIGHT_BACK_Pin, //
        ASEL1_Pin, ASEL2_Pin, //
        CHRG_EN_Pin> GpioInit;


    typedef GpioInitializer<        //
        MOT_A_HI_Pin, MOT_A_LO_Pin, //
        MOT_B_HI_Pin, MOT_B_LO_Pin, //
        //LIGHT_FRONT_Pin, LIGHT_BACK_Pin, //
        ASEL1_Pin, ASEL2_Pin, //
        REQ_BLOAD_Pin, //
        CHRG_EN_Pin> GpioBootloaderInit;

};


#endif // _ESP_DEADRAIL_PROTO_HARDWARE_HXX_
