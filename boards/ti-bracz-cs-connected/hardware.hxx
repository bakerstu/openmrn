

#ifndef _BRACZ_CS_TIVA_HARDWARE_HXX_
#define _BRACZ_CS_TIVA_HARDWARE_HXX_

#include "TivaGPIO.hxx"

/*
#define LED_RED GPIO_PORTN_BASE, GPIO_PIN_4
#define LED_YELLOW GPIO_PORTQ_BASE, GPIO_PIN_0
#define LED_GREEN GPIO_PORTP_BASE, GPIO_PIN_4
#define LED_BLUE GPIO_PORTD_BASE, GPIO_PIN_2 */

GPIO_PIN(LED_RED, LedPin, N, 4);
GPIO_PIN(LED_YELLOW, LedPin, Q, 0);
GPIO_PIN(LED_GREEN, LedPin, P, 4);
GPIO_PIN(LED_BLUE, LedPin, D, 2);

/*
#define LED_B1 GPIO_PORTN_BASE, GPIO_PIN_1
#define LED_B2 GPIO_PORTN_BASE, GPIO_PIN_0
#define LED_B3 GPIO_PORTF_BASE, GPIO_PIN_4
#define LED_B4 GPIO_PORTF_BASE, GPIO_PIN_0
*/

GPIO_PIN(LED_B1, LedPin, N, 1);
GPIO_PIN(LED_B2, LedPin, N, 0);
GPIO_PIN(LED_B3, LedPin, F, 4);
GPIO_PIN(LED_B4, LedPin, F, 0);


GPIO_PIN(USR_SW1, GpioInputPU, J, 0);
GPIO_PIN(USR_SW2, GpioInputPU, J, 1);

namespace io {
typedef LED_RED_Pin TrackPktLed;
typedef LED_BLUE_Pin TrackOnLed;
typedef LED_B1_Pin BlinkerLed;

typedef LED_YELLOW_Pin AccPwrLed;
typedef LED_B3_Pin GoPausedLed;

};


#endif // _BRACZ_CS_TIVA_HARDWARE_HXX_
