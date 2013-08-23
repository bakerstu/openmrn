/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file hw_init.cxx
 * low-level hardware initialization for the LPC2387 pandaII board.
 *
 * @author Balazs Racz
 * @date 13 April 2013
 */

#include <stdint.h>

#include "FreeRTOSConfig.h"
#include "plib.h"
#include "peripheral/ports.h"

//DigitalIn startpin(P1_4);

extern "C" {


void _cinit(void) {
  extern unsigned __cs3_regions[];
  memcpy((unsigned*)__cs3_regions[2], (unsigned*)__cs3_regions[1],
         __cs3_regions[3]);
  memset((unsigned*)(__cs3_regions[2] + __cs3_regions[3]), 0, __cs3_regions[4]);
}


#define SET_LED1() mPORTBSetBits(BIT_12)
#define CLR_LED1() mPORTBClearBits(BIT_12)

#define SET_LED2() mPORTBSetBits(BIT_15)
#define CLR_LED2() mPORTBClearBits(BIT_15)


void blocking_diewith_blinker(uint32_t pattern) {
  uint32_t curr_pat = pattern;
  while(1) {
    if (curr_pat & 1) {
      SET_LED1(); SET_LED2();
    } else {
      CLR_LED1(); CLR_LED2();
    }
    curr_pat>>=1;
    if (!curr_pat) curr_pat = pattern;
    // Some delay.
    volatile int i;
    for(i = 0; i < 80*1024; ++i);
  }
}

static unsigned int _excep_code; 
static unsigned int _excep_addr; 
static unsigned int _excep_vaddr;

void _general_exception_context(void)
{
  asm volatile("mfc0 %0,$8" : "=r" (_excep_vaddr)); 
  asm volatile("mfc0 %0,$13" : "=r" (_excep_code)); 
  asm volatile("mfc0 %0,$14" : "=r" (_excep_addr)); 
  
  blocking_diewith_blinker(0x8000A0CA); //3-1-2
}


void lowlevel_hw_init(void) {
  mPORTBSetPinsDigitalOut( BIT_12 | BIT_15 );

  // Configure the device for maximum performance but do not change PBDIV
  // Set the flash wait states, RAM wait state and enable prefetch cache,
  // but do NOT change PBDIV.
  // PBDIV is set from the configuration bytes during device reset.
  SYSTEMConfig(configCPU_CLOCK_HZ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

  // Enable the cache for the best performance (assuming we're running in KSEG0)
  CheKseg0CacheOn();
}

/** Initializes the processor hardware.
 */
void hw_init(void)
{


    mPORTBSetPinsDigitalOut( BIT_12 | BIT_15 );
    mPORTBSetBits(BIT_12 | BIT_15  );

  
    /*    LPC_GPIO3->FIODIR=(1<<26);
    LPC_GPIO3->FIOCLR=(1<<26);
    setblink(0x8000000AUL);
    // Waits for the start button to be pressed.
    while(!startpin)
    {
    }
    resetblink(1);*/
}

}  // extern C
