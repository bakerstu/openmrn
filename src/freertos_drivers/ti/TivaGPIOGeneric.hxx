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
 * \file TivaGPIOGeneric.hxx
 *
 * Helper declarations for using GPIO pins (both for GPIO and other hardware)
 * on Tiva MCUs.
 *
 * @author Stuart Baker
 * @date 1 July 2015
 */

#ifndef _FREERTOS_DRIVERS_TI_TIVAGPIOGENERIC_HXX_
#define _FREERTOS_DRIVERS_TI_TIVAGPIOGENERIC_HXX_

#include <stdint.h>

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "GPIOGeneric.hxx"

/** Tiva specific implementation of GPIO.  For the Tiva devices, the GPIO
 * number mapping can be found in the @ref GpioMapping enumeration.
 */
class TivaGpio : public Gpio
{
public:
    /** Constructor.
     * @param number GPIO number
     * @param mode GPIO mode settings
     * @param safe default "safe" value, may go unused for input only pins
     */
    TivaGpio(unsigned number, Mode mode, Value safe = CLR);

    /** Destructor.
     */
    ~TivaGpio()
    {
    }

    /** Test the GPIO pin to see if it is set.
     * @return true if currently a '1', false CLR if currently a '0'.
     *         Note: if the GPIO is inverted, this has the effect of
     *         returning the opposite value
     */
    bool is_set() OVERRIDE
    {
        bool set = (HWREG(base + (GPIO_O_DATA + (bit << 2))) != 0);
        return invert ? !set : set;
    }

    /** Test the GPIO pin to see if it is clear.
     * @return true if currently a '0', false CLR if currently a '1'.
     *         Note: if the GPIO is inverted, this has the effect of
     *         returning the opposite value
     */
    bool is_clr() OVERRIDE
    {
        bool clr = (HWREG(base + (GPIO_O_DATA + (bit << 2))) == 0);
        return invert ? !clr : clr;
    }

    /** Set the GPIO to a '1'.  Note: if the GPIO is inverted, this could
     * have the opposite effect of clearing the GPIO to a value of '0'.
     */
    void set() OVERRIDE
    {
        HWREG(base + (GPIO_O_DATA + (bit << 2))) = invert ? 0 : 0xFF;
    }

    /** Clear the GPIO to a '0'.  Note: if the GPIO is inverted, this has
     * have the opposite effect of setting the GPIO to a value of '1'.
     */
    void clr() OVERRIDE
    {
        HWREG(base + (GPIO_O_DATA + (bit << 2))) = invert ? 0xFF : 0;
    }

    /** Set the GPIO direction.
     * @param mode @ref INPUT or @ref OUTPUT
     */
    void direction(Mode mode) OVERRIDE;

    /** Get the GPIO direction.
     * @return @ref INPUT or @ref OUTPUT
     */
    Mode direction() OVERRIDE;

private:
    /** BASE address of the GPIO */
    uint32_t base;

    DISALLOW_COPY_AND_ASSIGN(TivaGpio);
};

/** Mapping of Tiva GPIO names found in the literature to generic gpio numbers.
 * This enumeration is here for convenience and should only be used in device
 * specific code.  The generic GPIO number should be used whenever accessing
 * generic code.
 */
enum TivaGpioMapping
{
    PORTA_0 = 0, /**< Port A bit 0 */
    PORTA_1, /**< Port A bit 1 */
    PORTA_2, /**< Port A bit 2 */
    PORTA_3, /**< Port A bit 3 */
    PORTA_4, /**< Port A bit 4 */
    PORTA_5, /**< Port A bit 5 */
    PORTA_6, /**< Port A bit 6 */
    PORTA_7, /**< Port A bit 7 */
    PORTB_0, /**< Port B bit 0 */
    PORTB_1, /**< Port B bit 1 */
    PORTB_2, /**< Port B bit 2 */
    PORTB_3, /**< Port B bit 3 */
    PORTB_4, /**< Port B bit 4 */
    PORTB_5, /**< Port B bit 5 */
    PORTB_6, /**< Port B bit 6 */
    PORTB_7, /**< Port B bit 7 */
    PORTC_0, /**< Port C bit 0 */
    PORTC_1, /**< Port C bit 1 */
    PORTC_2, /**< Port C bit 2 */
    PORTC_3, /**< Port C bit 3 */
    PORTC_4, /**< Port C bit 4 */
    PORTC_5, /**< Port C bit 5 */
    PORTC_6, /**< Port C bit 6 */
    PORTC_7, /**< Port C bit 7 */
    PORTD_0, /**< Port D bit 0 */
    PORTD_1, /**< Port D bit 1 */
    PORTD_2, /**< Port D bit 2 */
    PORTD_3, /**< Port D bit 3 */
    PORTD_4, /**< Port D bit 4 */
    PORTD_5, /**< Port D bit 5 */
    PORTD_6, /**< Port D bit 6 */
    PORTD_7, /**< Port D bit 7 */
    PORTE_0, /**< Port E bit 0 */
    PORTE_1, /**< Port E bit 1 */
    PORTE_2, /**< Port E bit 2 */
    PORTE_3, /**< Port E bit 3 */
    PORTE_4, /**< Port E bit 4 */
    PORTE_5, /**< Port E bit 5 */
    PORTE_6, /**< Port E bit 6 */
    PORTE_7, /**< Port E bit 7 */
    PORTF_0, /**< Port F bit 0 */
    PORTF_1, /**< Port F bit 1 */
    PORTF_2, /**< Port F bit 2 */
    PORTF_3, /**< Port F bit 3 */
    PORTF_4, /**< Port F bit 4 */
    PORTF_5, /**< Port F bit 5 */
    PORTF_6, /**< Port F bit 6 */
    PORTF_7, /**< Port F bit 7 */
    PORTG_0, /**< Port G bit 0 */
    PORTG_1, /**< Port G bit 1 */
    PORTG_2, /**< Port G bit 2 */
    PORTG_3, /**< Port G bit 3 */
    PORTG_4, /**< Port G bit 4 */
    PORTG_5, /**< Port G bit 5 */
    PORTG_6, /**< Port G bit 6 */
    PORTG_7, /**< Port G bit 7 */
    PORTH_0, /**< Port H bit 0 */
    PORTH_1, /**< Port H bit 1 */
    PORTH_2, /**< Port H bit 2 */
    PORTH_3, /**< Port H bit 3 */
    PORTH_4, /**< Port H bit 4 */
    PORTH_5, /**< Port H bit 5 */
    PORTH_6, /**< Port H bit 6 */
    PORTH_7, /**< Port H bit 7 */
    PORTJ_0, /**< Port J bit 0 */
    PORTJ_1, /**< Port J bit 1 */
    PORTJ_2, /**< Port J bit 2 */
    PORTJ_3, /**< Port J bit 3 */
    PORTJ_4, /**< Port J bit 4 */
    PORTJ_5, /**< Port J bit 5 */
    PORTJ_6, /**< Port J bit 6 */
    PORTJ_7, /**< Port J bit 7 */
    PORTK_0, /**< Port K bit 0 */
    PORTK_1, /**< Port K bit 1 */
    PORTK_2, /**< Port K bit 2 */
    PORTK_3, /**< Port K bit 3 */
    PORTK_4, /**< Port K bit 4 */
    PORTK_5, /**< Port K bit 5 */
    PORTK_6, /**< Port K bit 6 */
    PORTK_7, /**< Port K bit 7 */
    PORTL_0, /**< Port L bit 0 */
    PORTL_1, /**< Port L bit 1 */
    PORTL_2, /**< Port L bit 2 */
    PORTL_3, /**< Port L bit 3 */
    PORTL_4, /**< Port L bit 4 */
    PORTL_5, /**< Port L bit 5 */
    PORTL_6, /**< Port L bit 6 */
    PORTL_7, /**< Port L bit 7 */
    PORTM_0, /**< Port M bit 0 */
    PORTM_1, /**< Port M bit 1 */
    PORTM_2, /**< Port M bit 2 */
    PORTM_3, /**< Port M bit 3 */
    PORTM_4, /**< Port M bit 4 */
    PORTM_5, /**< Port M bit 5 */
    PORTM_6, /**< Port M bit 6 */
    PORTM_7, /**< Port M bit 7 */
    PORTN_0, /**< Port N bit 0 */
    PORTN_1, /**< Port N bit 1 */
    PORTN_2, /**< Port N bit 2 */
    PORTN_3, /**< Port N bit 3 */
    PORTN_4, /**< Port N bit 4 */
    PORTN_5, /**< Port N bit 5 */
    PORTN_6, /**< Port N bit 6 */
    PORTN_7, /**< Port N bit 7 */
    PORTP_0, /**< Port P bit 0 */
    PORTP_1, /**< Port P bit 1 */
    PORTP_2, /**< Port P bit 2 */
    PORTP_3, /**< Port P bit 3 */
    PORTP_4, /**< Port P bit 4 */
    PORTP_5, /**< Port P bit 5 */
    PORTP_6, /**< Port P bit 6 */
    PORTP_7, /**< Port P bit 7 */
    PORTQ_0, /**< Port Q bit 0 */
    PORTQ_1, /**< Port Q bit 1 */
    PORTQ_2, /**< Port Q bit 2 */
    PORTQ_3, /**< Port Q bit 3 */
    PORTQ_4, /**< Port Q bit 4 */
    PORTQ_5, /**< Port Q bit 5 */
    PORTQ_6, /**< Port Q bit 6 */
    PORTQ_7, /**< Port Q bit 7 */
    PORTR_0, /**< Port R bit 0 */
    PORTR_1, /**< Port R bit 1 */
    PORTR_2, /**< Port R bit 2 */
    PORTR_3, /**< Port R bit 3 */
    PORTR_4, /**< Port R bit 4 */
    PORTR_5, /**< Port R bit 5 */
    PORTR_6, /**< Port R bit 6 */
    PORTR_7, /**< Port R bit 7 */
    PORTS_0, /**< Port S bit 0 */
    PORTS_1, /**< Port S bit 1 */
    PORTS_2, /**< Port S bit 2 */
    PORTS_3, /**< Port S bit 3 */
    PORTS_4, /**< Port S bit 4 */
    PORTS_5, /**< Port S bit 5 */
    PORTS_6, /**< Port S bit 6 */
    PORTS_7, /**< Port S bit 7 */
    PORTT_0, /**< Port T bit 0 */
    PORTT_1, /**< Port T bit 1 */
    PORTT_2, /**< Port T bit 2 */
    PORTT_3, /**< Port T bit 3 */
    PORTT_4, /**< Port T bit 4 */
    PORTT_5, /**< Port T bit 5 */
    PORTT_6, /**< Port T bit 6 */
    PORTT_7, /**< Port T bit 7 */
    PIN_COUNT /**< Total number of GPIO pins */
};

#endif /* _FREERTOS_DRIVERS_TI_TIVAGPIOGENERIC_HXX_ */


