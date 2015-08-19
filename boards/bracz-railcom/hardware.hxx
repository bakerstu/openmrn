#ifndef _HARDWARE_HXX_
#define _HARDWARE_HXX_

#include "TivaGPIO.hxx"
#include "driverlib/rom_map.h"
#include "driverlib/timer.h"
#include "utils/GpioInitializer.hxx"
#include "inc/hw_ints.h"

GPIO_PIN(SW1, GpioInputPU, F, 4);
GPIO_PIN(SW2, GpioInputPU, F, 0);

GPIO_PIN(LED_RED_RAW, LedPin, F, 1);
GPIO_PIN(LED_GREEN, LedPin, F, 3);
GPIO_PIN(LED_BLUE, LedPin, F, 2);

typedef LED_RED_RAW_Pin BLINKER_RAW_Pin;

GPIO_HWPIN(UART0RX, GpioHwPin, A, 0, U0RX, UART);
GPIO_HWPIN(UART0TX, GpioHwPin, A, 1, U0TX, UART);

GPIO_HWPIN(CAN0RX, GpioHwPin, B, 4, CAN0RX, CAN);
GPIO_HWPIN(CAN0TX, GpioHwPin, E, 5, CAN0TX, CAN);

GPIO_HWPIN(RAILCOM_CH0, GpioHwPin, D, 6, U2RX, UART);
GPIO_HWPIN(RAILCOM_CH1, GpioHwPin, C, 6, U3RX, UART);
GPIO_HWPIN(RAILCOM_CH2, GpioHwPin, C, 4, U4RX, UART);
GPIO_HWPIN(RAILCOM_CH3, GpioHwPin, E, 0, U7RX, UART);
GPIO_HWPIN(RAILCOM_CH4, GpioHwPin, E, 4, U5RX, UART);
GPIO_HWPIN(RAILCOM_CH5, GpioHwPin, B, 0, U1RX, UART);

GPIO_PIN(OUTPUT_EN0, GpioOutputODSafeHigh, A, 3);
GPIO_PIN(OUTPUT_EN1, GpioOutputODSafeHigh, A, 4);
GPIO_PIN(OUTPUT_EN2, GpioOutputODSafeHigh, C, 7);
GPIO_PIN(OUTPUT_EN3, GpioOutputODSafeHigh, C, 5);
GPIO_PIN(OUTPUT_EN4, GpioOutputODSafeHigh, E, 1);
GPIO_PIN(OUTPUT_EN5, GpioOutputODSafeHigh, B, 5);

typedef LED_GREEN_Pin STAT0_Pin;
GPIO_PIN(STAT1, LedPin, E, 2);
GPIO_PIN(STAT2, LedPin, E, 3);
GPIO_PIN(STAT3, LedPin, A, 7);
GPIO_PIN(STAT4, LedPin, A, 6);
GPIO_PIN(STAT5, LedPin, A, 5);

GPIO_HWPIN(DCC_IN, GpioHwPin, B, 1, T2CCP1, Timer);

GPIO_HWPIN(DAC_TIMER, GpioHwPin, D, 7, WT5CCP1, Timer);
GPIO_PIN(DAC_DIV, GpioOutputSafeHigh, A, 2);

GPIO_PIN(GNDACTRL_NON, GpioOutputODSafeLow, D, 3);
GPIO_PIN(GNDACTRL_NOFF, GpioOutputSafeHigh, D, 2);
GPIO_PIN(GNDBCTRL_NON, GpioOutputODSafeHigh, D, 0);
GPIO_PIN(GNDBCTRL_NOFF, GpioOutputSafeLow, D, 1);

// These are internally connected to D0 and D1.
GPIO_PIN(SHADOW_1, GpioInputNP, B, 6);
GPIO_PIN(SHADOW_2, GpioInputNP, B, 7);

GPIO_PIN(RCBYPASS_NON, GpioOutputSafeLow, B, 3);
// GPIO_PIN(RCBYPASS_OFF, GpioOutputSafeLow, B, 2);
GPIO_PIN(RCBYPASS_OFF, LedPin, B, 2);

typedef GpioInitializer<                               //
    SW1_Pin, SW2_Pin,                                  //
    LED_RED_RAW_Pin, LED_GREEN_Pin, LED_BLUE_Pin,      //
    UART0RX_Pin, UART0TX_Pin,                          //
    RAILCOM_CH0_Pin, RAILCOM_CH1_Pin, RAILCOM_CH2_Pin, //
    RAILCOM_CH3_Pin, RAILCOM_CH4_Pin, RAILCOM_CH5_Pin, //
    OUTPUT_EN0_Pin, OUTPUT_EN1_Pin, OUTPUT_EN2_Pin,    //
    OUTPUT_EN3_Pin, OUTPUT_EN4_Pin, OUTPUT_EN5_Pin,    //
    STAT0_Pin, STAT1_Pin, STAT2_Pin,                   //
    STAT3_Pin, STAT4_Pin, STAT5_Pin,                   //
    DCC_IN_Pin,                                        //
    DAC_TIMER_Pin, DAC_DIV_Pin,                        //
    GNDACTRL_NON_Pin, GNDACTRL_NOFF_Pin,               //
    GNDBCTRL_NON_Pin, GNDBCTRL_NOFF_Pin,               //
    SHADOW_1_Pin, SHADOW_2_Pin,                        //
    RCBYPASS_OFF_Pin, RCBYPASS_NON_Pin,                //
    CAN0RX_Pin, CAN0TX_Pin> GpioInit;

typedef GpioInitializer<                            //
    SW1_Pin,                                        //
    LED_RED_RAW_Pin, LED_GREEN_Pin, LED_BLUE_Pin,   //
    OUTPUT_EN0_Pin, OUTPUT_EN1_Pin, OUTPUT_EN2_Pin, //
    OUTPUT_EN3_Pin, OUTPUT_EN4_Pin, OUTPUT_EN5_Pin, //
    GNDACTRL_NON_Pin, GNDACTRL_NOFF_Pin,            //
    GNDBCTRL_NON_Pin, GNDBCTRL_NOFF_Pin,            //
    SHADOW_1_Pin, SHADOW_2_Pin,                     //
    RCBYPASS_OFF_Pin, RCBYPASS_NON_Pin,             //
    CAN0RX_Pin, CAN0TX_Pin> BootloaderGpioInit;

#ifndef PINDEFS_ONLY

#include "DummyGPIO.hxx"

struct RailcomDefs
{
    static const uint32_t CHANNEL_COUNT = 6;
    static const uint32_t UART_PERIPH[CHANNEL_COUNT];
    static const uint32_t UART_BASE[CHANNEL_COUNT];
    // Make sure there are enough entries here for all the channels times a few
    // DCC packets.
    static const uint32_t Q_SIZE = 24;

    static const auto OS_INTERRUPT = INT_UART1;

    static void enable_measurement();
    static void disable_measurement();

    static void hw_init()
    {
        RAILCOM_CH0_Pin::hw_init();
        RAILCOM_CH1_Pin::hw_init();
        RAILCOM_CH2_Pin::hw_init();
        RAILCOM_CH3_Pin::hw_init();
        RAILCOM_CH4_Pin::hw_init();
        RAILCOM_CH5_Pin::hw_init();
    }

    static void set_input()
    {
        RAILCOM_CH0_Pin::set_input();
        RAILCOM_CH1_Pin::set_input();
        RAILCOM_CH2_Pin::set_input();
        RAILCOM_CH3_Pin::set_input();
        RAILCOM_CH4_Pin::set_input();
        RAILCOM_CH5_Pin::set_input();
    }

    static void set_hw()
    {
        RAILCOM_CH0_Pin::set_hw();
        RAILCOM_CH1_Pin::set_hw();
        RAILCOM_CH2_Pin::set_hw();
        RAILCOM_CH3_Pin::set_hw();
        RAILCOM_CH4_Pin::set_hw();
        RAILCOM_CH5_Pin::set_hw();
    }

    /** @returns a bitmask telling which pins are active. Bit 0 will be set if
     * channel 0 is active (drawing current).*/
    static uint8_t sample()
    {
        uint8_t ret = 0;
        if (!RAILCOM_CH0_Pin::get())
            ret |= 1;
        if (!RAILCOM_CH1_Pin::get())
            ret |= 2;
        if (!RAILCOM_CH2_Pin::get())
            ret |= 4;
        if (!RAILCOM_CH3_Pin::get())
            ret |= 8;
        if (!RAILCOM_CH4_Pin::get())
            ret |= 16;
        if (!RAILCOM_CH5_Pin::get())
            ret |= 32;
        return ret;
    }
};

#define RAILCOM_BASE                                                           \
    {                                                                          \
        UART2_BASE, UART3_BASE, UART4_BASE, UART7_BASE, UART5_BASE, UART1_BASE \
    }

#define RAILCOM_PERIPH                                                         \
    {                                                                          \
        SYSCTL_PERIPH_UART2, SYSCTL_PERIPH_UART3, SYSCTL_PERIPH_UART4,         \
            SYSCTL_PERIPH_UART7, SYSCTL_PERIPH_UART5, SYSCTL_PERIPH_UART1      \
    }

struct DCCDecode
{
    static const auto TIMER_BASE = TIMER2_BASE;
    static const auto TIMER_PERIPH = SYSCTL_PERIPH_TIMER2;
    static const auto TIMER_INTERRUPT = INT_TIMER2B;
    static const auto TIMER = TIMER_B;
    static const auto CFG_CAP_TIME_UP =
        TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_CAP_TIME_UP | TIMER_CFG_A_PWM;
    // Interrupt bits.
    static const auto TIMER_CAP_EVENT = TIMER_CAPB_EVENT;
    static const auto TIMER_TIM_TIMEOUT = TIMER_TIMB_TIMEOUT;

    static const auto OS_INTERRUPT = INT_TIMER2A;
    typedef DCC_IN_Pin NRZ_Pin;

    // 16-bit timer max + use 7 bits of prescaler.
    static const uint32_t TIMER_MAX_VALUE = 0x800000UL;
    static const uint32_t PS_MAX = 0x80;

    static const int Q_SIZE = 32;
};

struct DACDefs
{
    // If false, the timerconfigure call will be skipped. This is needed when
    // two halves of the same timer is shared between different devices.
    static const bool config_timer = true;
    static const auto TIMER_BASE = WTIMER5_BASE;
    static const auto TIMER_PERIPH = SYSCTL_PERIPH_WTIMER5;
    static const auto TIMER = TIMER_B;
    static const auto TIMER_CFG =
        TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PWM | TIMER_CFG_A_PWM;

    typedef DAC_TIMER_Pin TIMER_Pin;
    typedef DAC_DIV_Pin DIV_Pin;
};

struct DACDefsxx
{
    // If false, the timerconfigure call will be skipped. This is needed when
    // two halves of the same timer is shared between different devices.
    static const bool config_timer = true;
    static const auto TIMER_BASE = WTIMER1_BASE;
    static const auto TIMER_PERIPH = SYSCTL_PERIPH_WTIMER1;
    static const auto TIMER = TIMER_B;
    static const auto TIMER_CFG =
        TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PWM | TIMER_CFG_A_PWM;

    GPIO_HWPIN(TIMER, GpioHwPin, C, 7, WT1CCP1, Timer);

    // typedef DAC_TIMER_Pin TIMER_Pin;
    typedef DAC_DIV_Pin DIV_Pin;
};

// extern TivaDAC<DACDefs> dac;

struct Debug
{
    // One for the duration of the first byte successfully read from the uart
    // device for railcom until the next 1 msec when the receiver gets disabled.
    typedef DummyPin RailcomUartReceived;
    // Measures the time from the end of a DCC packet until the entry of the
    // user-space state flow.
    typedef DummyPin DccPacketDelay;

    // High between start_cutout and end_cutout from the TivaRailcom driver.
    // typedef DBG_SIGNAL_Pin RailcomDriverCutout;
    typedef DummyPin RailcomDriverCutout;

    // Flips every timer capture interrupt from the dcc deocder flow.
    // typedef LED_BLUE_SW_Pin DccDecodeInterrupts;
    typedef DummyPin DccDecodeInterrupts;

    // Flips every timer capture interrupt from the dcc deocder flow.
    // typedef DBG_SIGNAL_Pin RailcomE0;
    typedef LED_GREEN_Pin RailcomE0;

    // Flips every timer capture interrupt from the dcc deocder flow.
    typedef DummyPin RailcomError;
    // typedef LED_BLUE_SW_Pin RailcomError;

    typedef DummyPin RailcomDataReceived;
    typedef DummyPin RailcomAnyData;
    typedef DummyPin RailcomPackets;

    typedef DummyPin DetectRepeat;
};

#endif // ! pindefs_only

#endif // _HARDWARE_HXX_
