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
 * Based on example code published by ARM, Code Red, FreeRTOS and mbed.
 *
 * \file  boot.S
 * Startup code for the LPC23xx processors.
 *
 * @author Balazs Racz
 * @date 12 April 2013
 */

	.extern main
	.extern exit

	.text
	.code 32


	.align  0

	.extern __bss_beg__
	.extern __bss_end__
	.extern __stack_end__
	.extern __data_beg__
	.extern __data_end__
	.extern __data+beg_src__
	.extern __SRAM_segment_start__
	.extern __SRAM_segment_end__
	.extern __und_stack_top__
	.extern __abt_stack_top__
	.extern __fiq_stack_top__
	.extern __irq_stack_top__
	.extern __svc_stack_top__
	.extern __sys_stack_top__
	
	.global start
	.global endless_loop
	.global entry

	/* Stack Sizes */
    .set  UND_STACK_SIZE, 0x00000004
    .set  ABT_STACK_SIZE, 0x00000004
    .set  FIQ_STACK_SIZE, 0x00000004
    .set  IRQ_STACK_SIZE, 0X00000800
    .set  SVC_STACK_SIZE, 0x00000800

	/* Standard definitions of Mode bits and Interrupt (I & F) flags in PSRs */
    .set  MODE_USR, 0x10            /* User Mode */
    .set  MODE_FIQ, 0x11            /* FIQ Mode */
    .set  MODE_IRQ, 0x12            /* IRQ Mode */
    .set  MODE_SVC, 0x13            /* Supervisor Mode */
    .set  MODE_ABT, 0x17            /* Abort Mode */
    .set  MODE_UND, 0x1B            /* Undefined Mode */
    .set  MODE_SYS, 0x1F            /* System Mode */

    .equ  I_BIT, 0x80               /* when I bit is set, IRQ is disabled */
    .equ  F_BIT, 0x40               /* when F bit is set, FIQ is disabled */


	.set BLINK_DIE_DABT,0x8000A0CA    /* 3-1-2 */
	.set BLINK_DIE_PABT,0x800020CA    /* 3-1-1 */
	.set BLINK_DIE_UNDF,0x8002A0CA    /* 3-1-3 */
	.set BLINK_DIE_SVCIRQ,0x800AA0CA    /* 3-1-4 */
	
start:
_start:
_mainCRTStartup:
	
	//Fill entire stack segment with debug bits
        LDR   r0,=0xdadadada            // debug pattern
        LDR   r1,=__stacks_min__
        LDR   r2,=__stacks_top__ 
3:      CMP   r1,r2                     // check if data to clear
        STRLO r0,[r1],#4                // clear 4 bytes
        BLO   3b                        // loop until done


	/* Setup a stack for each mode - note that this only sets up a usable stack
	for system/user, SWI and IRQ modes.   Also each mode is setup with
	interrupts initially disabled. */
	msr   CPSR_c, #MODE_UND|I_BIT|F_BIT /* Undefined Instruction Mode */
	ldr   sp, =__und_stack_top__

	msr   CPSR_c, #MODE_ABT|I_BIT|F_BIT /* Abort Mode */
	ldr   sp, =__abt_stack_top__

	msr   CPSR_c, #MODE_FIQ|I_BIT|F_BIT /* FIQ Mode */
	ldr   sp, =__fiq_stack_top__
	bl __init_blinker
	
	msr   CPSR_c, #MODE_IRQ|I_BIT|F_BIT /* IRQ Mode */
	ldr   sp, =__irq_stack_top__

	msr   CPSR_c, #MODE_SVC|I_BIT|F_BIT /* Supervisor Mode */
	ldr   sp, =__svc_stack_top__

	msr   CPSR_c, #MODE_SYS|I_BIT|F_BIT /* System Mode */
	ldr   sp, =__sys_stack_top__

	/* We want to start in supervisor mode.  Operation will switch to system
	mode when the first task starts. */
	msr   CPSR_c, #MODE_SVC|I_BIT|F_BIT

	/* turn on debug LED */
	mov r1, #(1<<26)
	ldr r2, =0x3FFFC060 // FIO3DIR
	str r1, [r2]
	ldr r2, =0x3FFFC078 // FIO3SET
	//str r1, [r2]

	//bkpt


	//Fill entire ram segment with debug bits
        LDR   r0,=0xdbdbdbdb            // debug pattern
        LDR   r1,=__SRAM_segment_start__
        LDR   r2,=__SRAM_segment_end__ 
3:      CMP   r1,r2                     // check if data to clear
        STRLO r0,[r1],#4                // clear 4 bytes
        BLO   3b                        // loop until done


	
	//Initialize data segment
        LDR   r1,=__data_init_start__   // -> ROM data start
        LDR   r2,=__data_start__        // -> data start
        LDR   r3,=__data_end__          // -> end of data
1:      CMP   r2,r3                     // check if data to move
        LDRLO r0,[r1],#4                // copy it
        STRLO r0,[r2],#4
        BLO   1b                        // loop until done

	//Clear .bss (zero'ed space)
        MOV   r0,#0                     // get a zero
        LDR   r1,=__bss_start__         // -> bss start
        LDR   r2,=__bss_end__           // -> bss end
2:      CMP   r1,r2                     // check if data to clear
        STRLO r0,[r1],#4                // clear 4 bytes
        BLO   2b                        // loop until done

        ;
        //LDR R0, =__mbed_init_realmonitor
        //MOV LR, PC
        //BX R0

        ;
        LDR R0, =SystemInit
        MOV     lr,pc
        BX R0

        LDR R0, =__libc_init_array
        MOV     lr,pc
        BX R0

	mov r1, #(1<<26)
	ldr r2, =0x3FFFC078 // FIO3SET
	//str r1, [r2]

        LDR R0, =main
        BX R0

	
#if 0
	/* Clear BSS. */

	mov     a2, #0			/* Fill value */
	mov		fp, a2			/* Null frame pointer */
	mov		r7, a2			/* Null frame pointer for Thumb */

	ldr		r1, .LC1		/* Start of memory block */
	ldr		r3, .LC2		/* End of memory block */
	subs	r3, r3, r1      /* Length of block */
	beq		.end_clear_loop
	mov		r2, #0

.clear_loop:
	strb	r2, [r1], #1
	subs	r3, r3, #1
	bgt		.clear_loop

.end_clear_loop:

	/* Initialise data. */

	ldr		r1, .LC3		/* Start of memory block */
	ldr		r2, .LC4		/* End of memory block */
	ldr		r3, .LC5
	subs	r3, r3, r1		/* Length of block */
	beq		.end_set_loop

.set_loop:
	ldrb	r4, [r2], #1
	strb	r4, [r1], #1
	subs	r3, r3, #1
	bgt		.set_loop

.end_set_loop:

	
        LDR R0, =SystemInit
        MOV     lr,pc
        BX R0



	LDR R0, =__libc_init_array
	orr	r0, r0, #1
        MOV     lr,pc
        BX R0

	mov r1, #(1<<26)
	ldr r2, =0x3FFFC078 // FIO3SET
	//str r1, [r2]


	mov		r0, #0          /* no arguments  */
	mov		r1, #0          /* no argv either */

        mov	r2, =main
        MOV     lr,pc
	bx		main

endless_loop:
	b               endless_loop


	.align 0

	.LC1:
	.word   __bss_beg__
	.LC2:
	.word   __bss_end__
	.LC3:
	.word   __data_beg__
	.LC4:
	.word   __data_beg_src__
	.LC5:
	.word   __data_end__
	.LC6:
	.word   __stack_end__
#endif

	/* Setup vector table.  Note that undf, pabt, dabt, fiq just execute
	a null loop. */

.section .vectors,"ax"
         .code 32
         .align 0
entry:
	
	b     _start						/* reset - _start			*/
	ldr   pc, _undf						/* undefined - _undf		*/
	ldr   pc, _swi						/* SWI - _swi				*/
	ldr   pc, _pabt						/* program abort - _pabt	*/
	ldr   pc, _dabt						/* data abort - _dabt		*/
	nop									/* reserved					*/
	ldr   pc, [pc,#-0x120]				/* IRQ - read the VIC		*/
	ldr   pc, _fiq						/* FIQ - _fiq				*/

_undf:  .word __undf                    /* undefined				*/
_swi:   .word vPortYieldProcessor       /* SWI						*/
_pabt:  .word __pabt                    /* program abort			*/
_dabt:  .word __dabt                    /* data abort				*/
_fiq:   .word __fiq                     /* FIQ						*/

	.global __svcirq_crash
__svcirq_crash:
	push {lr}
	bl	enable_fiq
	ldr	r0, =BLINK_DIE_SVCIRQ
	ldr	lr, =blinker_pattern
	str	r0, [lr]
	pop	{lr}
	pop	{r0}   /* pushed by caller in portSAVE_CONTEXT */
	b     .                         /* undefined				*/


__undf:
	push {r0, lr}
	bl	enable_fiq
	ldr	r0, =BLINK_DIE_UNDF
	ldr	lr, =blinker_pattern
	str	r0, [lr]
	pop	{r0, lr}
	b     .                         /* undefined				*/
__pabt:
	push {r0, lr}
	bl	enable_fiq
	ldr	r0, =BLINK_DIE_PABT
	ldr	lr, =blinker_pattern
	str	r0, [lr]
	pop	{r0, lr}
	b     .                         /* program abort			*/
__dabt:
	push {r0, lr}
	bl	enable_fiq
	ldr	r0, =BLINK_DIE_DABT
	ldr	lr, =blinker_pattern
	str	r0, [lr]
	pop	{r0, lr}
	b     .                         /* data abort
*/
	subs	pc,lr,#0
__fiq:
	// First we set the output bit
	ldr	r9,=(1<<26)
	tst	r11, #1
	strne	r9, [r10, #0x18]
	streq	r9, [r10, #0x1C]
	// Shift bit right
	movs	r11, r11, LSR #1
	// If zero, load original bit pattern
	ldreq	r11,[r8]
	// Clear IRQ
	mov	r9, #1
	str	r9, [r12]
	// iret
	subs	pc, lr, #4
	
	
__init_blinker:
	ldr   r8, =blinker_pattern  // address of where to read the pattern from
	ldr   r9, =(1<<26)	// bit to flip
	ldr   r10, =0x3FFFC060 // FIO3 Base
	str   r9, [r10]
	mov   r11, #0	// pattern_tmp
	ldr   r12, =0xE0008000  // timr1 irq register


	bx lr
	
	.global	enable_fiq
enable_fiq:
	push	{r0}
	mrs  r0, CPSR
	bic  r0, r0, #0x40
	msr  CPSR, r0
	pop	{r0}
	bx	lr

	.global	enable_fiq_only
enable_fiq_only:
	push	{r0}
	mrs  r0, CPSR
	bic  r0, r0, #0x40
	orr  r0, r0, #0x80
	msr  CPSR, r0
	pop	{r0}
	bx	lr


// Reserves space for RAM interrupt vectors.
        .section .ram_vectors, "w"
        .space 64, 0
	
.section  .bss
	.global blinker_pattern
blinker_pattern:	
	.word	0
