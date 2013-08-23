/*********************************************************************
 *
 *                  pCache API definitions
 *
 *********************************************************************
 * FileName:        pCache.c
 * Dependencies:	p32xxxx.h
 * Processor:       PIC32MX
 *
 * Complier:        C32
 *
 * Company:         Microchip Technology Inc..
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
 * $Id: pCache.h,v 1.5 2006/10/23 18:07:00 C13128 Exp $
 *
 ********************************************************************/
#include <p32xxxx.h>

/*********************************************************************
 * Function:        void CheKseg0CacheOff(void)
 *
 * PreCondition:    None
 * Input:           None
 * Output:          none
 * Side Effects:    Sets the CCA field in the Config register of Co-
 * 					processor 0 to the value "010"b
 * Overview:        This routine is used to disable cacheability of KSEG0.
 *
 * Note:
 *
 ********************************************************************/
void __attribute__ ((nomips16)) CheKseg0CacheOff(void)
{
	register unsigned long tmp;

	asm("mfc0 %0,$16,0" :  "=r"(tmp));
	tmp = (tmp & ~7) | 2;
	asm("mtc0 %0,$16,0" :: "r" (tmp));
}


/*********************************************************************
 * Function:        void cheKseg0CacheOn(void)
 *
 * PreCondition:    None
 * Input:           None
 * Output:          none
 * Side Effects:    Sets the CCA field in the Config register of Co-
 * 					processor 0 to the value "011"b
 * Overview:        This routine is used to enable cacheability of KSEG0.
 *
 * Note:
 *
 ********************************************************************/
void __attribute__ ((nomips16)) CheKseg0CacheOn(void)
{
	register unsigned long tmp;
	asm("mfc0 %0,$16,0" :  "=r"(tmp));
	tmp = (tmp & ~7) | 3;
	asm("mtc0 %0,$16,0" :: "r" (tmp));
}
