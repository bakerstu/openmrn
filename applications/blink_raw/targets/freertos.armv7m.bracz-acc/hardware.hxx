#ifndef _ACC_TIVA_3_HARDWARE_HXX_
#define _ACC_TIVA_3_HARDWARE_HXX_

#define USE_WII_CHUCK

#include "TivaGPIO.hxx"
#include "DummyGPIO.hxx"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/timer.h"

GPIO_PIN(LED_RED, LedPin, D, 6);
GPIO_PIN(LED_GREEN, LedPin, D, 5);
GPIO_PIN(LED_BLUE, LedPin, G, 1);
GPIO_PIN(LED_YELLOW, LedPin, B, 0);

GPIO_PIN(LED_BLUE_SW, LedPin, B, 6);
GPIO_PIN(LED_GOLD_SW, LedPin, B, 7);

//GPIO_PIN(DBG_SIGNAL, GpioOutputSafeLow, B, 1);

GPIO_PIN(REL0, GpioOutputSafeLow, F, 2);
GPIO_PIN(REL1, GpioOutputSafeLow, F, 1);
GPIO_PIN(REL2, GpioOutputSafeLow, G, 5);
GPIO_PIN(REL3, GpioOutputSafeLow, F, 3);

GPIO_PIN(OUT0, GpioOutputSafeLow, E, 4);
GPIO_PIN(OUT1, GpioOutputSafeLow, E, 5);
GPIO_PIN(OUT2, GpioOutputSafeLow, D, 0);
GPIO_PIN(OUT3, GpioOutputSafeLow, D, 1);
GPIO_PIN(OUT4, GpioOutputSafeLow, D, 2);
GPIO_PIN(OUT5, GpioOutputSafeLow, D, 3);
GPIO_PIN(OUT6, GpioOutputSafeLow, E, 3);
GPIO_PIN(OUT7, GpioOutputSafeLow, E, 2);

GPIO_PIN(IN0, GpioInputNP, A, 0);
GPIO_PIN(IN1, GpioInputNP, A, 1);
GPIO_PIN(IN2, GpioInputNP, A, 2);
GPIO_PIN(IN3, GpioInputNP, A, 3);
GPIO_PIN(IN4, GpioInputNP, A, 4);
GPIO_PIN(IN5, GpioInputNP, A, 5);
GPIO_PIN(IN6, GpioInputNP, A, 6);
GPIO_PIN(IN7, GpioInputNP, A, 7);

GPIO_PIN(BUT_BLUE, GpioInputPU, C, 6);
GPIO_PIN(BUT_GOLD, GpioInputPU, C, 7);

GPIO_PIN(RC4_SHADOW, GpioInputNP, F, 0);


typedef LED_RED_Pin BlinkerLed;

struct Debug {
  // One for the duration of the first byte successfully read from the uart
  // device for railcom until the next 1 msec when the receiver gets disabled.
  typedef DummyPin RailcomUartReceived;
  // Measures the time from the end of a DCC packet until the entry of the
  // user-space state flow.
  typedef DummyPin DccPacketDelay;

  // High between start_cutout and end_cutout from the TivaRailcom driver.
  //typedef DBG_SIGNAL_Pin RailcomDriverCutout;
  typedef DummyPin RailcomDriverCutout;

  // Flips every timer capture interrupt from the dcc deocder flow.
  //typedef LED_BLUE_SW_Pin DccDecodeInterrupts;
  typedef DummyPin DccDecodeInterrupts;
 
  // Flips every timer capture interrupt from the dcc deocder flow.
  //typedef DBG_SIGNAL_Pin RailcomE0;
  typedef LED_GREEN_Pin RailcomE0;

  // Flips every timer capture interrupt from the dcc deocder flow.
  typedef DummyPin RailcomError;
  //typedef LED_BLUE_SW_Pin RailcomError;

    typedef DummyPin RailcomDataReceived;
    typedef DummyPin RailcomAnyData;
    typedef DummyPin RailcomPackets;


  typedef LED_GOLD_SW_Pin DetectRepeat;
};

struct DCCDecode
{
    static const auto TIMER_BASE = WTIMER4_BASE;
    static const auto TIMER_PERIPH = SYSCTL_PERIPH_WTIMER4;
    static const auto TIMER_INTERRUPT = INT_WTIMER4A;
    static const auto TIMER = TIMER_A;
    static const auto CFG_CAP_TIME_UP =
        TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP | TIMER_CFG_B_ONE_SHOT;
    // Interrupt bits.
    static const auto TIMER_CAP_EVENT = TIMER_CAPA_EVENT;
    static const auto TIMER_TIM_TIMEOUT = TIMER_TIMA_TIMEOUT;

    static const auto OS_INTERRUPT = INT_WTIMER4B;
    DECL_HWPIN(NRZPIN, D, 4, WT4CCP0, Timer);

    static const uint32_t TIMER_MAX_VALUE = 0x8000000UL;
    static const uint32_t PS_MAX = 0;

    static const int Q_SIZE = 32;
};

#define HAVE_RAILCOM

struct RailcomHw
{
    static const uint32_t CHANNEL_COUNT = 4;
    static const uint32_t UART_PERIPH[CHANNEL_COUNT];
    static const uint32_t UART_BASE[CHANNEL_COUNT];
    // Make sure there are enough entries here for all the channels times a few
    // DCC packets.
    static const uint32_t Q_SIZE = 16;

    static const auto OS_INTERRUPT = INT_UART2;

    GPIO_HWPIN(CH1, GpioHwPin, C, 4, U4RX, UART);
    GPIO_HWPIN(CH2, GpioHwPin, C, 6, U3RX, UART);
    GPIO_HWPIN(CH3, GpioHwPin, G, 4, U2RX, UART);
    GPIO_HWPIN(CH4, GpioHwPin, E, 0, U7RX, UART);

    static void hw_init() {
         CH1_Pin::hw_init();
         CH2_Pin::hw_init();
         CH3_Pin::hw_init();
         CH4_Pin::hw_init();
    }

    static void set_input() {
        CH1_Pin::set_input();
        CH2_Pin::set_input();
        CH3_Pin::set_input();
        CH4_Pin::set_input();
    }

    static void set_hw() {
        CH1_Pin::set_hw();
        CH2_Pin::set_hw();
        CH3_Pin::set_hw();
        CH4_Pin::set_hw();
    }

    /// @returns a bitmask telling which pins are active. Bit 0 will be set if
    /// channel 0 is active (drawing current).
    static uint8_t sample() {
        uint8_t ret = 0;
        if (!CH1_Pin::get()) ret |= 1;
        if (!CH2_Pin::get()) ret |= 2;
        if (!CH3_Pin::get()) ret |= 4;
        if (!CH4_Pin::get()) ret |= 8;
        return ret;
    }
};

const uint32_t RailcomHw::UART_BASE[] __attribute__((weak)) = {UART4_BASE, UART3_BASE, UART2_BASE, UART7_BASE};
const uint32_t RailcomHw::UART_PERIPH[]
__attribute__((weak)) = {SYSCTL_PERIPH_UART4, SYSCTL_PERIPH_UART3, SYSCTL_PERIPH_UART2, SYSCTL_PERIPH_UART7};

#if NODEID_LOW_BITS == 0x4E
//#define USE_WII_CHUCK
#endif

#endif // _ACC_TIVA_3_HARDWARE_HXX_
