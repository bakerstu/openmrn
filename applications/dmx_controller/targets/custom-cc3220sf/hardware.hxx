#ifndef _HARDWARE_HXX_
#define _HARDWARE_HXX_

#include "CC3200GPIO.hxx"
#include "BlinkerGPIO.hxx"
#include "driverlib/rom_map.h"
#include "utils/GpioInitializer.hxx"
#include "driverlib/adc.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin.h"
#include "driverlib/timer.h"
#include "inc/hw_timer.h"
#include "inc/hw_ints.h"

GPIO_PIN(SW1, GpioInputPin, A3, 0);

GPIO_PIN(LED_2, GpioOutputSafeLow, A1, 0);
GPIO_PIN(LED_3, GpioOutputSafeLow, A0, 0);
GPIO_PIN(LED_4, GpioOutputSafeLow, A3, 6);
GPIO_PIN(LED_5, GpioOutputSafeLow, A2, 7);

GPIO_PIN(MCP2515_RESET_N, GpioOutputSafeLow,  A0, 6);
GPIO_PIN(MCP2515_CS_N,    GpioOutputSafeHigh, A2, 1);
GPIO_PIN(MCP2515_INT_N,   GpioInputPin,       A0, 7);

GPIO_PIN(RS485_TX_EN, GpioOutputSafeLow, A2, 6);

GPIO_PIN(RESVD1, GpioOutputSafeLow, A0, 3);
GPIO_PIN(RESVD2, GpioOutputSafeLow, A3, 4);
GPIO_PIN(RESVD3, GpioOutputSafeLow, A1, 2);
GPIO_PIN(RESVD4, GpioOutputSafeLow, A1, 3);
GPIO_PIN(RESVD5, GpioOutputSafeLow, A0, 4);

using REQ_BLOAD_Pin = SW1_Pin;
using BLINKER_RAW_Pin = LED_4_Pin;
constexpr auto BLINKER_TIMER_BASE = TIMERA1_BASE;
constexpr auto BLINKER_TIMER = TIMER_A;
constexpr auto BLINKER_TIMER_TIMEOUT = TIMER_TIMA_TIMEOUT;
constexpr auto BLINKER_TIMER_INT = INT_TIMERA1A;

// Create an initializer that can initialize all the GPIO pins in one shot
typedef GpioInitializer<SW1_Pin,                                    //
                        LED_2_Pin, LED_3_Pin, LED_4_Pin, LED_5_Pin, //
                        MCP2515_RESET_N_Pin,                        //
                        MCP2515_CS_N_Pin,                           //
                        MCP2515_INT_N_Pin,                          //
                        RS485_TX_EN_Pin,                            //
                        RESVD1_Pin, RESVD2_Pin, //
                        RESVD3_Pin, RESVD4_Pin, //
                        RESVD5_Pin> GpioInit;

template<int N> void init_pinmux() {
    /* initilize pin modes:
     *   PIN_01:  PIN_MODE_0 - GPIO10     RESVD3
     *   PIN_02:  PIN_MODE_0 - GPIO11     RESVD4
     *   PIN_03:  PIN_MODE_7 - UART0_TX   RS-485_RX
     *   PIN_04:  PIN_MODE_7 - UART0_RX   RS-485_RX
     *   PIN_05:  PIN_MODE_7 - GSPI_CLK   SPI_SCLK
     *   PIN_06:  PIN_MODE_7 - GSPI_MISO  SPI_MISO
     *   PIN_07:  PIN_MODE_7 - GSPI_MOSI  SPI_MOSI
     *   PIN_08:  PIN_MODE_0 - GPIO17     SPI_CS_N
     *   PIN_15:  PIN_MODE_0 - GPIO22     RS-485_TX_ENABLE
     *   PIN_16:  PIN_MODE_0 - GPIO23     LED_5
     *   PIN_17:  PIN_MODE_0 - GPIO24     BUTTON
     *   PIN_18:  PIN_MODE_0 - GPIO28     RESVD1
     *   PIN_50:  PIN_MODE_0 - GPIO0      LED_3
     *   PIN_53:  PIN_MODE_0 - GPIO30     LED_4
     *   PIN_55:  PIN_MODE_0 - GPIO1      BOOTLOADER UART0_TX
     *   PIN_57:  PIN_MODE_0 - GPIO2      BOOTLOADER UART0_RX
     *   PIN_58:  PIN_MODE_0 - GPIO3      RESVD1
     *   PIN_59:  PIN_MODE_0 - GPIO4      RESVD5
     *   PIN_60:  PIN_MODE_0 - ADC_CH3    ANALOG
     *   PIN_61:  PIN_MODE_0 - GPIO6      CAN_RESET_N
     *   PIN_62:  PIN_MODE_0 - GPIO7      CAN_INT_N
     *   PIN_63:  PIN_MODE_0 - GPIO8      LED_2
     *   PIN_64:  PIN_MODE_3 - GT_PWM05   CAN_CLOCK
     */
    MAP_PinTypeTimer(PIN_01, PIN_MODE_3);
    MAP_PinTypeTimer(PIN_02, PIN_MODE_3);
    MAP_PinTypeUART( PIN_03, PIN_MODE_7);
    MAP_PinTypeUART( PIN_04, PIN_MODE_7);
    MAP_PinTypeSPI(  PIN_05, PIN_MODE_7);
    MAP_PinTypeSPI(  PIN_06, PIN_MODE_7);
    MAP_PinTypeSPI(  PIN_07, PIN_MODE_7);
    MAP_PinTypeGPIO( PIN_08, PIN_MODE_0, false);
    MAP_PinTypeGPIO( PIN_15, PIN_MODE_0, false);
    MAP_PinTypeGPIO( PIN_16, PIN_MODE_0, false);
    MAP_PinTypeGPIO( PIN_17, PIN_MODE_0, false);
    MAP_PinTypeGPIO( PIN_18, PIN_MODE_0, false);
    MAP_PinTypeGPIO( PIN_50, PIN_MODE_0, false);
    MAP_PinTypeGPIO( PIN_53, PIN_MODE_0, false);
    //MAP_PinTypeUART( PIN_55, PIN_MODE_3);
    //MAP_PinTypeUART( PIN_57, PIN_MODE_3);
    MAP_PinTypeGPIO( PIN_55, PIN_MODE_0, false);
    MAP_PinTypeGPIO( PIN_57, PIN_MODE_0, false);
    MAP_PinTypeGPIO( PIN_58, PIN_MODE_0, false);
    MAP_PinTypeGPIO( PIN_59, PIN_MODE_0, false);
    MAP_PinTypeADC(  PIN_60, PIN_MODE_255);
    MAP_PinTypeGPIO( PIN_61, PIN_MODE_0, false);
    MAP_PinTypeGPIO( PIN_62, PIN_MODE_0, false);
    MAP_PinTypeGPIO( PIN_63, PIN_MODE_0, false);
    MAP_PinTypeTimer(PIN_64, PIN_MODE_3);

    // enable WPU on the hardware button.
    MAP_PinConfigSet( PIN_17, PIN_STRENGTH_2MA, PIN_TYPE_STD_PU);

    // enable WPU on the RS-485 RX.
    MAP_PinConfigSet( PIN_04, PIN_STRENGTH_2MA, PIN_TYPE_STD_PU);
}


#endif // _HARDWARE_HXX_
