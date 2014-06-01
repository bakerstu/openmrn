/*********************************************************************
 *
 *                  Power API definitions
 *
 *********************************************************************
 * FileName:        power.h
 * Dependencies:
 * Processor:     PIC32
 *
 *
 * Compiler:        MPLAB XC32
 *                  MPLAB IDE
 * Company:         Microchip Technology Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PIC Microcontroller is intended
 * and supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PIC Microcontroller products.
 * The software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *
 * $Id: OSC.h,v 1.6 2006/10/13 21:24:31 C12532 Exp $
 *
 * $Name:  $
 ********************************************************************/

//#include <xc.h>
//#include <plib.h>
#include <peripheral/osc.h>

/*********************************************************************
 * Function:        OSCConfig(source)
 *
 * Description:	    Sets Osc options andclock source
 *
 * PreCondition:    Interrupts must be disabled
 *
 * Inputs:	    Clock source, PLL multiplier, PLL postscaler, FRC divisor
 *
 * Output:          None
 *
 * Example:	    OSCSelect( OSC_POSC, OSC_PLL_MULT_24, OSC_PLL_POST_256, OSC_FRC_POST_64 )
 *
 * Note:	    Forces cpu clock source to FRC(no divisor, no PLL), configures new clock
 * 		     source and then switches to the new clock source
 *
 *		    unused parameters can be set to zero.
 ********************************************************************/
void OSCConfig(unsigned long int source, unsigned long int mult, unsigned long int post, unsigned long int div)
{
	int	intStat;
	int	dmaSusp;

	source &= _OSCCON_NOSC_MASK;			// mask passed parameters
	mult &= _OSCCON_PLLMULT_MASK;
	post &= _OSCCON_PLLODIV_MASK;
	div &= _OSCCON_FRCDIV_MASK;


	mSYSTEMUnlock(intStat, dmaSusp);

	// unlock OSSCON register, set new clock source to FRC, request clock switch
	OSCCONCLR = _OSCCON_NOSC_MASK;
	OSCCONSET = _OSCCON_OSWEN_MASK;

	while ( _OSCCON_OSWEN_MASK & OSCCON) {}; 	// wait for the clock switch to FRC

	OSCCONCLR = _OSCCON_PLLMULT_MASK | _OSCCON_PLLODIV_MASK | _OSCCON_NOSC_MASK | _OSCCON_FRCDIV_MASK; // clear the current settings
	OSCCONSET = (mult) | (post) | (source) | (div);	// set the new configuration

	OSCCONSET = _OSCCON_OSWEN_MASK;	// request clock switch

	while ( _OSCCON_OSWEN_MASK & OSCCON) {}; 	// wait for the clock switch to FRC

	mSYSTEMLock(intStat, dmaSusp);	// restore status

}


