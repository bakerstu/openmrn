#ifndef _ESP_BRACZ_DEADRAIL_HUZZAH_HARDWARE_HXX_
#define _ESP_BRACZ_DEADRAIL_HUZZAH_HARDWARE_HXX_

#include "freertos_drivers/esp8266/Esp8266Gpio.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"
#include "utils/GpioInitializer.hxx"

#error baar

struct HW
{
    GPIO_PIN(BLINKER_RAW, GpioOutputSafeHigh, 0);
    static constexpr bool blinker_invert = true;

    GPIO_PIN(MOT_A_HI, GpioOutputSafeLow, 4);
    GPIO_PIN(MOT_A_LO, GpioOutputSafeLow, 5);

    GPIO_PIN(MOT_B_HI, GpioOutputSafeLow, 14);
    GPIO_PIN(MOT_B_LO, GpioOutputSafeLow, 12);


    static constexpr bool invertLow = false;

    // forward: A=HI B=LO

    // typedef BLINKER_Pin LIGHT_FRONT_Pin;
    typedef DummyPin LIGHT_FRONT_Pin;
    //GPIO_PIN(LIGHT_FRONT, GpioOutputSafeLow, 13);
    typedef DummyPin LIGHT_BACK_Pin;
    //GPIO_PIN(LIGHT_BACK, GpioOutputSafeLow, 5);

    // Doubles as manual request pin.
    GPIO_PIN(REQ_BLOAD, GpioInputPU, 13);

    GPIO_PIN(F1, GpioOutputSafeHighInvert, 2);
    typedef F1_Pin BootloaderActivePin;

    typedef GpioInitializer<        //
        MOT_A_HI_Pin, MOT_A_LO_Pin, //
        MOT_B_HI_Pin, MOT_B_LO_Pin, //
        LIGHT_FRONT_Pin, LIGHT_BACK_Pin, //
        BLINKER_RAW_Pin, //
//        REQ_BLOAD_Pin,
        F1_Pin> GpioInit;


    typedef GpioInitializer<        //
        MOT_A_HI_Pin, MOT_A_LO_Pin, //
        MOT_B_HI_Pin, MOT_B_LO_Pin, //
        BLINKER_RAW_Pin, F1_Pin, //
        REQ_BLOAD_Pin> GpioBootloaderInit;

};


#endif // _ESP_DEADRAIL_HUZZAH_HARDWARE_HXX_
