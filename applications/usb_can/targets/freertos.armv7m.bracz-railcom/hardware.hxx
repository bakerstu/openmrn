#ifndef _HARDWARE_HXX_
#define _HARDWARE_HXX_

#include "TivaGPIO.hxx"
#include "driverlib/rom_map.h"
#include "driverlib/timer.h"
#include "utils/GpioInitializer.hxx"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"

// note : this might cause problems in the bootloader compilation
#include "DummyGPIO.hxx"


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

//typedef LED_GREEN_Pin STAT0_Pin;
//typedef DummyPin STAT0_Pin;
GPIO_PIN(DEBUG1, GpioOutputSafeLow, E, 2);
GPIO_PIN(DEBUG2, GpioOutputSafeLow, E, 3);
// Charlieplexing pins.
GPIO_PIN(CHARLIE0, LedPin, A, 7);
GPIO_PIN(CHARLIE1, LedPin, A, 6);
GPIO_PIN(CHARLIE2, LedPin, A, 5);

GPIO_HWPIN(DCC_IN, GpioHwPin, B, 1, T2CCP1, Timer);

GPIO_HWPIN(DAC_TIMER, GpioHwPin, D, 7, WT5CCP1, Timer);
GPIO_PIN(DAC_DIV, GpioOutputSafeHigh, A, 2);

GPIO_PIN(GNDACTRL_NON, GpioOutputODSafeLow, D, 3);
GPIO_PIN(GNDACTRL_NOFF, GpioOutputODSafeHigh, D, 2);
//GPIO_PIN(GNDBCTRL_NON, GpioOutputODSafeHigh, D, 0);
GPIO_PIN(GNDBCTRL_NON, GpioOutputODSafeHigh, D, 0);
GPIO_PIN(GNDBCTRL_NOFF, GpioOutputSafeLow, D, 1);

// These are internally connected to D0 and D1.
GPIO_PIN(SHADOW_1, GpioOutputODSafeHigh, B, 6);
GPIO_PIN(SHADOW_2, GpioOutputODSafeHigh, B, 7);

GPIO_PIN(RCBYPASS_NON, GpioOutputSafeLow, B, 3);
// GPIO_PIN(RCBYPASS_OFF, GpioOutputSafeLow, B, 2);
GPIO_PIN(RCBYPASS_OFF, LedPin, B, 2);

typedef GpioInitializer<                                //
    SW1_Pin, SW2_Pin,                                   //
    LED_RED_RAW_Pin, LED_GREEN_Pin, LED_BLUE_Pin,       //
    UART0RX_Pin, UART0TX_Pin,                           //
    RAILCOM_CH0_Pin, RAILCOM_CH1_Pin, RAILCOM_CH2_Pin,  //
    RAILCOM_CH3_Pin, RAILCOM_CH4_Pin, RAILCOM_CH5_Pin,  //
    OUTPUT_EN0_Pin, OUTPUT_EN1_Pin, OUTPUT_EN2_Pin,     //
    OUTPUT_EN3_Pin, OUTPUT_EN4_Pin, OUTPUT_EN5_Pin,     //
    CHARLIE0_Pin, CHARLIE1_Pin, CHARLIE2_Pin,           //
    DEBUG1_Pin, DEBUG2_Pin,                             //
    DCC_IN_Pin,                                         //
    DAC_TIMER_Pin, DAC_DIV_Pin,                         //
    GNDACTRL_NON_Pin, GNDACTRL_NOFF_Pin,                //
    GNDBCTRL_NON_Pin, GNDBCTRL_NOFF_Pin,                //
    SHADOW_1_Pin, SHADOW_2_Pin,                         //
    RCBYPASS_OFF_Pin, RCBYPASS_NON_Pin,                 //
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

struct Debug
{
    // One for the duration of the first byte successfully read from the uart
    // device for railcom until the next 1 msec when the receiver gets disabled.
    typedef DummyPin RailcomUartReceived;
    // Measures the time from the end of a DCC packet until the entry of the
    // user-space state flow.
    typedef DummyPin DccPacketDelay;

    // Set to one when we receive data on the 2nd channel of railcom.
    typedef DEBUG2_Pin RailcomCh2Data;

    // High between start_cutout and end_cutout from the TivaRailcom driver.
    // typedef DBG_SIGNAL_Pin RailcomDriverCutout;
    typedef DEBUG1_Pin RailcomDriverCutout;

    // Flips every timer capture interrupt from the dcc deocder flow.
    // typedef LED_BLUE_SW_Pin DccDecodeInterrupts;
    typedef LED_GREEN_Pin DccDecodeInterrupts;

    // Flips every timer capture interrupt from the dcc deocder flow.
    // typedef DBG_SIGNAL_Pin RailcomE0;
    //typedef LED_GREEN_Pin RailcomE0;
    typedef DummyPin RailcomE0;

    // Flips every timer capture interrupt from the dcc deocder flow.
    typedef DummyPin RailcomError;
    // typedef LED_BLUE_SW_Pin RailcomError;

    typedef DummyPin RailcomDataReceived;

    //typedef LED_BLUE_Pin RailcomDataReceived;
    typedef DummyPin RailcomAnyData;
    typedef DummyPin RailcomPackets;

    typedef DummyPin DetectRepeat;


    //typedef DummyPin MeasurementEnabled;
    typedef LED_BLUE_Pin MeasurementEnabled;

    typedef DummyPin NSampling;

};

namespace TDebug {
    typedef DummyPin Resync;
    typedef DummyPin NextPacket;
};

struct RailcomDefs
{
    static const uint32_t CHANNEL_COUNT = 6;
    static const uint32_t UART_PERIPH[CHANNEL_COUNT];
    static const uint32_t UART_BASE[CHANNEL_COUNT];
    // Make sure there are enough entries here for all the channels times a few
    // DCC packets.
    static const uint32_t Q_SIZE = 24;

    static const auto OS_INTERRUPT = INT_UART1;

    static bool need_ch1_cutout() {
        return true;
    }

    static void enable_measurement();
    static void disable_measurement();

    static uint8_t feedbackChannel_;
    static uint8_t get_feedback_channel() {
        return feedbackChannel_;
    }
    static void set_feedback_channel(uint8_t arg) {
        feedbackChannel_ = arg;
    }

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
        RAILCOM_CH0_Pin::set_input(GPIO_PIN_TYPE_STD_WPU);
        RAILCOM_CH1_Pin::set_input(GPIO_PIN_TYPE_STD_WPU);
        RAILCOM_CH2_Pin::set_input(GPIO_PIN_TYPE_STD_WPU);
        RAILCOM_CH3_Pin::set_input(GPIO_PIN_TYPE_STD_WPU);
        RAILCOM_CH4_Pin::set_input(GPIO_PIN_TYPE_STD_WPU);
        RAILCOM_CH5_Pin::set_input(GPIO_PIN_TYPE_STD_WPU);
    }

    static void set_hw()
    {
        GPIODirModeSet(RAILCOM_CH0_Pin::GPIO_BASE, RAILCOM_CH0_Pin::GPIO_PIN,
                       GPIO_DIR_MODE_HW);
        GPIODirModeSet(RAILCOM_CH1_Pin::GPIO_BASE, RAILCOM_CH1_Pin::GPIO_PIN,
                       GPIO_DIR_MODE_HW);
        GPIODirModeSet(RAILCOM_CH2_Pin::GPIO_BASE, RAILCOM_CH2_Pin::GPIO_PIN,
                       GPIO_DIR_MODE_HW);
        GPIODirModeSet(RAILCOM_CH3_Pin::GPIO_BASE, RAILCOM_CH3_Pin::GPIO_PIN,
                       GPIO_DIR_MODE_HW);
        GPIODirModeSet(RAILCOM_CH4_Pin::GPIO_BASE, RAILCOM_CH4_Pin::GPIO_PIN,
                       GPIO_DIR_MODE_HW);
        GPIODirModeSet(RAILCOM_CH5_Pin::GPIO_BASE, RAILCOM_CH5_Pin::GPIO_PIN,
                       GPIO_DIR_MODE_HW);
/*        RAILCOM_CH0_Pin::set_hw();
        GPIOPadConfigSet(
            RAILCOM_CH0_Pin::GPIO_BASE, RAILCOM_CH0_Pin::GPIO_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

        RAILCOM_CH1_Pin::set_hw();
        GPIOPadConfigSet(
            RAILCOM_CH1_Pin::GPIO_BASE, RAILCOM_CH1_Pin::GPIO_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

        RAILCOM_CH2_Pin::set_hw();
        GPIOPadConfigSet(
            RAILCOM_CH2_Pin::GPIO_BASE, RAILCOM_CH2_Pin::GPIO_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

        RAILCOM_CH3_Pin::set_hw();
        GPIOPadConfigSet(
            RAILCOM_CH3_Pin::GPIO_BASE, RAILCOM_CH3_Pin::GPIO_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

        RAILCOM_CH4_Pin::set_hw();
        GPIOPadConfigSet(
            RAILCOM_CH4_Pin::GPIO_BASE, RAILCOM_CH4_Pin::GPIO_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

        RAILCOM_CH5_Pin::set_hw();
        GPIOPadConfigSet(
        RAILCOM_CH5_Pin::GPIO_BASE, RAILCOM_CH5_Pin::GPIO_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);*/
    }

    /** @returns a bitmask telling which pins are active. Bit 0 will be set if
     * channel 0 is active (drawing current).*/
    static uint8_t sample()
    {
        uint8_t ret = 0;
        if (feedbackChannel_ == 0xff) {
            extern volatile uint32_t sample_count;
            sample_count++;
        }
        Debug::NSampling::set(false);
        MAP_SysCtlDelay(5*26);
        if (!RAILCOM_CH0_Pin::get()) {
            extern volatile uint32_t ch0_count;
            if (feedbackChannel_ == 0xff) {
                ch0_count++;
            }
            ret |= 1;
        }
        if (!RAILCOM_CH1_Pin::get()) {
            extern volatile uint32_t ch1_count;
            if (feedbackChannel_ == 0xff) {
                ch1_count++;
            }
            ret |= 2;
        }
        if (!RAILCOM_CH2_Pin::get())
            ret |= 4;
        if (!RAILCOM_CH3_Pin::get())
            ret |= 8;
        if (!RAILCOM_CH4_Pin::get())
            ret |= 16;
        if (!RAILCOM_CH5_Pin::get())
            ret |= 32;
        Debug::NSampling::set(true);

        /*if (feedbackChannel_ == 0xff) {
          if (ret & 1) {
            CHARLIE0_Pin::instance()->set_direction(Gpio::Direction::OUTPUT);
            CHARLIE0_Pin::set(true);
          } else {
            CHARLIE0_Pin::instance()->set_direction(Gpio::Direction::INPUT);
          }

          if (ret & 2) {
            CHARLIE1_Pin::instance()->set_direction(Gpio::Direction::OUTPUT);
            CHARLIE1_Pin::set(true);
          } else {
            CHARLIE1_Pin::instance()->set_direction(Gpio::Direction::INPUT);
          }
          }*/

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

struct DCCDecode
{
    static const auto TIMER_BASE = TIMER2_BASE;
    static const auto TIMER_PERIPH = SYSCTL_PERIPH_TIMER2;
    static const auto TIMER_INTERRUPT = INT_TIMER2B;
    static const auto TIMER = TIMER_B;
    static const auto CFG_TIM_CAPTURE =
        TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_CAP_TIME;
    static const auto CFG_RCOM_TIMER =
        TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC;

    // Interrupt bits.
    static const auto TIMER_CAP_EVENT = TIMER_CAPB_EVENT;
    static const auto TIMER_RCOM_MATCH = TIMER_TIMA_MATCH;

    // Sets the match register of TIMER to update immediately.
    static void clr_tim_mrsu() {
      HWREG(TIMER_BASE + TIMER_O_TAMR) &= ~(TIMER_TAMR_TAMRSU);
      HWREG(TIMER_BASE + TIMER_O_TAMR) |= (TIMER_TAMR_TAMIE);
    }

    static const auto RCOM_TIMER = TIMER_A;
    static const auto SAMPLE_PERIOD_CLOCKS = 60000;
    //static const auto SAMPLE_TIMER_TIMEOUT = TIMER_TIMA_TIMEOUT;
    static const auto RCOM_INTERRUPT = INT_TIMER2A;
    //static const auto OS_INTERRUPT = INT_TIMER2A;
    typedef DCC_IN_Pin NRZ_Pin;

    // 16-bit timer max + use 7 bits of prescaler.
    static const uint32_t TIMER_MAX_VALUE = 0x800000UL;
    static const uint32_t PS_MAX = 0x80;

    static_assert(SAMPLE_PERIOD_CLOCKS < TIMER_MAX_VALUE, "Cannot sample less often than the timer period");

    static const int Q_SIZE = 32;

    // after 5 overcurrent samples we get one occupancy sample
    static const uint32_t OCCUPANCY_SAMPLE_RATIO = 5;
    static inline void dcc_before_cutout_hook();
    static inline void dcc_packet_finished_hook();
    static inline void after_feedback_hook();
};

#endif // ! pindefs_only

#endif // _HARDWARE_HXX_
