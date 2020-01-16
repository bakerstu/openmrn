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
#include <string.h>

#include "utils/macros.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "plib.h"
#include "peripheral/ports.h"
#include "peripheral/timer.h"
#include "utils/blinker.h"

//DigitalIn startpin(P1_4);

extern "C" {

const unsigned long pic32_cpu_clock_hz = 40000000UL;
const unsigned long pic32_periph_clock_hz = 40000000UL;

void _cinit(void) {
  extern unsigned __cs3_regions[];
  memcpy((unsigned*)__cs3_regions[2], (unsigned*)__cs3_regions[1],
         __cs3_regions[3]);
  memset((unsigned*)(__cs3_regions[2] + __cs3_regions[3]), 0, __cs3_regions[4]);
}


#define SET_LED1() mPORTDSetBits(BIT_1)     // Gold
#define CLR_LED1() mPORTDClearBits(BIT_1)

#define SET_LED2() mPORTDSetBits(BIT_2)     // Blue
#define CLR_LED2() mPORTDClearBits(BIT_2)

void diewith(uint32_t pattern) {
    setblink(pattern);
    while(1);
}

uint32_t blinker_pattern = 0;
static uint32_t rest_pattern = 0;

void __attribute__((interrupt, nomips16)) tmr2_interrupt(void)
{
    // Set output LED.
    if (rest_pattern & 1) {
        CLR_LED2();
    } else {
        SET_LED2();
    }
    rest_pattern >>= 1;
    if (!rest_pattern)
    {
        rest_pattern = blinker_pattern;
    }
    INTClearFlag(INT_T2);
}

asm("\n\t.section .vector_8,\"ax\",%progbits\n\tj "
    "tmr2_interrupt\n\tnop\n.text\n");

static void __attribute__((nomips16)) enable_blinker() {
    // Adds a filter to not get interrupts at freertos priorities.
    portDISABLE_INTERRUPTS();
    // This will enable hardware interrupts.
    INTEnableInterrupts();
    portDISABLE_INTERRUPTS();
}

void setblink(uint32_t pattern) {
    enable_blinker();
    resetblink(pattern);
}

void resetblink(uint32_t pattern) {
    blinker_pattern = pattern;
    // triggers an int right now.
    WriteTimer2(ReadPeriod2() - 10);
}

static unsigned int _excep_code; 
static unsigned int _excep_addr; 
static unsigned int _excep_vaddr;

void __attribute__((nomips16)) _general_exception_context(void)
{
  asm volatile("mfc0 %0,$8" : "=r" (_excep_vaddr)); 
  asm volatile("mfc0 %0,$13" : "=r" (_excep_code)); 
  asm volatile("mfc0 %0,$14" : "=r" (_excep_addr)); 
  
  diewith(0x8000A0CA); //3-1-2
}

void hw_preinit(void) 
{
  mPORTDSetPinsDigitalOut( BIT_1 | BIT_2 );
  mPORTDClearBits( BIT_1 | BIT_2 );         // Blue and Gold LED ON

  mPORTFSetPinsDigitalOut( BIT_4 | BIT_5 );
  mPORTFSetBits  ( BIT_4 );                 // Red LED OFF
  mPORTFClearBits( BIT_5 );                 // Green LED ON


  // We want 8 ticks per second.
  OpenTimer2(T2_ON | T2_IDLE_CON | T2_GATE_OFF | T2_PS_1_256 | T2_32BIT_MODE_OFF | T2_SOURCE_INT, configPERIPHERAL_CLOCK_HZ / 256 / 8);
  ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_6 | T2_INT_SUB_PRIOR_0);
  
  setblink(0x8000CA);

  // Sets the main clock ot 40 MHz through PLL. 
  // Not needed here, configuration bits are set in pic32mx_cfg_init.c  (robh)
  //OSCConfig(OSC_POSC_PLL, OSC_PLL_MULT_15, OSC_PLL_POST_1, OSC_FRC_POST_1); ????
  //OSCConfig(OSC_FRC_PLL, OSC_PLL_MULT_15, OSC_PLL_POST_1, OSC_FRC_POST_1);  ????
  HASSERT(configCPU_CLOCK_HZ == 40000000);

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
    mPORTDSetBits( BIT_1 | BIT_2 );         // Blue and Gold LED OFF
	
    mPORTFClearBits( BIT_4 );               // Red LED ON
}

}  // extern C
