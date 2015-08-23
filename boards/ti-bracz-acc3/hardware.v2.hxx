#ifndef _ACC_TIVA_2_HARDWARE_HXX_
#define _ACC_TIVA_2_HARDWARE_HXX_

#include "TivaGPIO.hxx"
#include "DummyGPIO.hxx"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/timer.h"

GPIO_PIN(LED_RED_RAW, LedPin, D, 6);
GPIO_PIN(LED_GREEN, LedPin, D, 5);
GPIO_PIN(LED_BLUE, LedPin, G, 1);
GPIO_PIN(LED_YELLOW, LedPin, B, 0);

GPIO_PIN(LED_BLUE_SW, LedPin, B, 6);
GPIO_PIN(LED_GOLD_SW, LedPin, B, 7);

//GPIO_PIN(DBG_SIGNAL, GpioOutputSafeLow, B, 1);

GPIO_PIN(REL0, GpioOutputSafeLow, C, 4);
GPIO_PIN(REL1, GpioOutputSafeLow, C, 5);
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

typedef LED_RED_RAW_Pin BlinkerLed;

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

#endif // _ACC_TIVA_2_HARDWARE_HXX_
