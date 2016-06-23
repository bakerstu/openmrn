#ifndef _ESP_BRACZ_DEADRAIL_PROTO_HARDWARE_HXX_
#define _ESP_BRACZ_DEADRAIL_PROTO_HARDWARE_HXX_

#include "freertos_drivers/esp8266/Esp8266Gpio.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"

struct HW
{
    /* original / standard definitions.
        GPIO_PIN(MOT_A_HI, GpioOutputSafeLow, 4);
        GPIO_PIN(MOT_A_LO, GpioOutputSafeLow, 5);

        GPIO_PIN(MOT_B_HI, GpioOutputSafeLow, 14);
        GPIO_PIN(MOT_B_LO, GpioOutputSafeLow, 12);

        // forward: A=HI B=LO

        GPIO_PIN(LIGHT_FRONT, GpioOutputSafeLow, 13);
        GPIO_PIN(LIGHT_BACK, GpioOutputSafeLow, 15);

        GPIO_PIN(F1, GpioOutputSafeLow, 2);

        //typedef DummyPin F1_Pin;

    */

    GPIO_PIN(BLINKER_RAW, GpioOutputSafeHigh, 0);
    static constexpr bool blinker_invert = true;

    GPIO_PIN(MOT_A_HI, GpioOutputSafeLow, 14);
    GPIO_PIN(MOT_A_LO, GpioOutputSafeLow, 12);

    GPIO_PIN(MOT_B_HI, GpioOutputSafeLow, 4);
    GPIO_PIN(MOT_B_LO, GpioOutputSafeLow, 13);

    static constexpr bool invertLow = false;

    // forward: A=HI B=LO

    // typedef BLINKER_Pin LIGHT_FRONT_Pin;
    typedef DummyPin LIGHT_FRONT_Pin;
    //GPIO_PIN(LIGHT_FRONT, GpioOutputSafeLow, 13);
    typedef DummyPin LIGHT_BACK_Pin;
    //GPIO_PIN(LIGHT_BACK, GpioOutputSafeLow, 5);

    // Doubles as manual request pin.
    GPIO_PIN(REQ_BLOAD, GpioInputPU, 5);

    GPIO_PIN(F1, GpioOutputSafeHigh, 2);
    // typedef DummyPin F1_Pin;

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


#endif // _ESP_DEADRAIL_PROTO_HARDWARE_HXX_
