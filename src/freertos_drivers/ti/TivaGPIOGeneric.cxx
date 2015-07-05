/** \copyright
 * Copyright (c) 2014, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file TivaGPIOGeneric.cxx
 *
 * Helper declarations for using GPIO pins (both for GPIO and other hardware)
 * on Tiva MCUs.
 *
 * @author Stuart Baker
 * @date 1 July 2015
 */

//#include <stdlib.h>

#include "driverlib/rom_map.h"
#include "driverlib/rom.h"

/** Constructor.
 * @param GPIO number
 * @param mode GPIO mode settings
 * @param safe default "safe" value, may go unused for input only pins
 */
TivaGpio::TivaGpio(unsigned number, Mode mode, Value safe)
    : Gpio(number, mode, safe)
    , base(0)
{
    /* test for plausibly valid GPIO number */
    HASSERT(number < PIN_COUNT);
    /* pull up and pull down should be mutually exclusive */
    HASSERT((mode & (PULL_UP | PULL_DOWN)) == (PULL_UP | PULL_DOWN));

    /* we override the base class default of 0 */
    bit = 0x01 << (number % 8);

    /* setup port specific attributes */
    if (number <= PORTA_7)
    {
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        base = GPIO_PORTA_BASE;
    }
    else if (number <= PORTB_7)
    {
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        base = GPIO_PORTB_BASE;
    }
    else if (number <= PORTC_7)
    {
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
        base = GPIO_PORTC_BASE;
    }
    else if (number <= PORTD_7)
    {
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
        base = GPIO_PORTD_BASE;
    }
    else if (number <= PORTE_7)
    {
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
        base = GPIO_PORTE_BASE;
    }
    else if (number <= PORTF_7)
    {
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        base = GPIO_PORTF_BASE;
    }
    else if (number <= PORTG_7)
    {
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
        base = GPIO_PORTG_BASE;
    }
    else if (number <= PORTH_7)
    {
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
        base = GPIO_PORTH_BASE;
    }
    else if (number <= PORTJ_7)
    {
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
        base = GPIO_PORTJ_BASE;
    }
    else if (number <= PORTK_7)
    {
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
        base = GPIO_PORTK_BASE;
    }
    else if (number <= PORTL_7)
    {
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
        base = GPIO_PORTL_BASE;
    }
    else if (number <= PORTM_7)
    {
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
        base = GPIO_PORTM_BASE;
    }
    else if (number <= PORTN_7)
    {
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
        base = GPIO_PORTN_BASE;
    }
    else if (number <= PORTP_7)
    {
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
        base = GPIO_PORTP_BASE;
    }
    else if (number <= PORTQ_7)
    {
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
        base = GPIO_PORTQ_BASE;
    }
    else if (number <= PORTR_7)
    {
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOR);
        base = GPIO_PORTR_BASE;
    }
    else if (number <= PORTS_7)
    {
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOS);
        base = GPIO_PORTS_BASE;
    }
    else if (number <= PORTT_7)
    {
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOT);
        base = GPIO_PORTT_BASE;
    }

    /* configure for input/output */
    if (mode & OUTPUT)
    {
        MAP_GPIOPinTypeGPIOOutput(base, bit);
        MAP_GPIOPinWrite(base, bit, (safeValue ^ invert) ? bit : 0);
    }
    else
    {
        MAP_GPIOPinTypeGPIOInput(base, bit);        
    }

    /* configure drive stregnth and type */
    uint32_t strength = mode & LED ? GPIO_STRENGTH_8MA : GPIO_STRENGTH_2MA;
    uint32_t type;
    if (mode & PULL_UP)
    {
        type = GPIO_PIN_TYPE_STD_WPU;
    }
    else if (mode & PULL_DOWN)
    {
        type = GPIO_PIN_TYPE_STD_WPD;
    }
    else
    {
        type = GPIO_PIN_TYPE_STD;
    } 
    MAP_GPIOPadConfigSet(base, bit, strength, type);

    /* regiseter the GPIO pin number with our tracking mechanism */
    track();
}

/** Set the GPIO direction.
 * @param mode @ref INPUT or @ref OUTPUT
 */
void TivaGpio::direction(Mode mode)
{
    HASSERT(mode == INPUT || mode == OUTPUT);
    MAP_GPIODirModeSet(base, bit, mode == INPUT ? GPIO_DIR_MODE_IN :
                                                          GPIO_DIR_MODE_OUT);
}

/** Get the GPIO direction.
 * @return @ref INPUT or @ref OUTPUT
 */
Gpio::Mode TivaGpio::direction()
{
    uint32_t mode = MAP_GPIODirModeGet(base, bit);
    switch (mode)
    {
        default:
            HASSERT(0);
        case GPIO_DIR_MODE_IN:
            return INPUT;
        case GPIO_DIR_MODE_OUT:
            return OUTPUT;
    }
}
