/* Reset code for PIC32MX
**
** Prepare hardware, and then branch to C startup code.
** Sets up multi-vectored interrupts.
** WARNING: Interdependent with Nadler LD script and CodeSourcery CS3 init.
**
** Not implemented:
**   - BEV (bootstrap exception) processing
**   - general exception processing
**   - NMI trap
**   - bootstrap loader interface (app signature validation etc)
**   - initialization of shadow register set for use by ISRs
**
** For reference see:
**   - Microchip PIC32MX_Family_Reference_Manual document 61132B.pdf
**   - CodeSourcery "Malta 24k" startup code
**   - Microchip startup code crt0.s
**
** Copyright (c) 2009 Dave Nadler
**
** The authors hereby grant permission to use, copy, modify, distribute,
** and license this software and its documentation for any purpose, provided
** that existing copyright notices are retained in all copies and that this
** notice is included verbatim in any distributions.  No written agreement,
** license, or royalty fee is required for any of the authorized uses.
** Modifications to this software may be copyrighted by their authors
** and need not follow the licensing terms described here, provided that
** the new terms are clearly indicated on the first page of each file where
** they apply.
**
** History:
**  15Jul09 DRNadler: Save PIC32MX reset reason code for app, then clear it.
**  11May09 DRNadler: Initial implementation
*/

#include "p32xxxx.h"

# data saved below, NOT to be initialized (overwritten) by CS3 startup code
	.section .reset_data
	.globl _SAVED_RCON
_SAVED_RCON:
	.word 0

	.section .cs3.reset,"ax",%progbits
	.globl __cs3_reset_PIC32MX
	.type	__cs3_reset_PIC32MX, %function

__cs3_reset_PIC32MX:


#	===== Skip NMI trap for now =====
#
#	##################################################################
#	# If entered because of an NMI, jump to the NMI handler.
#	##################################################################
#	mfc0	k0,_CP0_STATUS
#	ext	k0,k0,19,1		# Extract NMI bit
#	beqz	k0,_no_nmi
#	nop
#	la	k0,_nmi_handler
#	jr	k0
#	nop
# _no_nmi:


	##################################################################
	# Initialize Stack Pointer
	#	_stack is initialized by the linker script to point to the
	#	 starting location of the stack in DRM
	##################################################################
	la		sp,__cs3_stack

	##################################################################
	# Initialize Global Pointer
	#	_gp is initialized by the linker script to point to "middle"
	#	of the small variables region
	##################################################################
	la		gp,_gp

	##################################################################
	# Save the reset reason code for later use
	lw		t1,RCON
	sw		t1,_SAVED_RCON
	# clear reset reason code bits so subsequent reset gets updated info
	xor		t1,t1,t1
	ori		t1,t1,0xFEFF # Clear everything but VREG bit
	sw		t1,RCONCLR
	##################################################################

	##################################################################
	# Initialize Global Pointer in Shadow Set
	#	The SRSCtl's PSS field must be set to the shadow set in which
	#	to initialize the global pointer.  Since we only have a
	#	single shadow set (besides the normal), we will initialize
	#	SRSCtl<PSS> to SRSCtl<HSS>.	 We then write the global pointer
	#	to the previous shadow set to ensure that on interrupt, the
	#	global pointer has been initialized.
	##################################################################
	mfc0	t1,_CP0_SRSCTL	# Read SRSCtl register
	add		t3,t1,zero		# Save off current SRSCtl
	ext		t2,t1,26,4		# to obtain HSS field
	ins		t1,t2,6,4		# Put HSS field
	mtc0	t1,_CP0_SRSCTL	# into SRSCtl<PSS>
	wrpgpr	gp,gp			# Set global pointer in PSS
	mtc0	t3,_CP0_SRSCTL	# Restore SRSCtl

	##################################################################
	# Initialize CP0 registers
	##################################################################
	# Initialize Count register
	##################################################################
	mtc0	zero,_CP0_COUNT

	##################################################################
	# Initialize Compare register
	##################################################################
	li		t2,-1
	mtc0	t2,_CP0_COMPARE

	##################################################################
	# Initialize CAUSE registers
	# - Enable counting of Count register <DC = 0>
	# - Use special exception vector <IV = 1>
	# - Clear pending software interrupts <IP1:IP0 = 0>
	##################################################################
	li		t1,0x00800000
	mtc0	t1,_CP0_CAUSE

	##################################################################
	# Initialize STATUS register
	# - Access to Coprocessor 0 not allowed in user mode <CU0 = 0>
	# - User mode uses configured endianness <RE = 0>
	# - Preserve Bootstrap Exception vectors <BEV>
	# - Preserve soft reset <SR> and non-maskable interrupt <NMI>
	# - CorExtend enabled based on whether CorExtend User Defined
	#	Instructions have been implemented <CEE = Config<UDI>>
	# - Disable any pending interrups <IM7..IM2 = 0, IM1..IM0 = 0>
	# - Disable hardware interrupts <IPL7:IPL2 = 0>
	# - Base mode is Kernel mode <UM = 0>
	# - Error level is normal <ERL = 0>
	# - Exception level is normal <EXL = 0>
	# - Interrupts are disabled <IE = 0>
	##################################################################
	mfc0	t0,_CP0_CONFIG
	ext		t1,t0,22,1		# Extract UDI from Config register
	sll		t1,t1,17		# Move UDI to Status.CEE location
	mfc0	t0,_CP0_STATUS
	and		t0,t0,0x00580000	# Preserve SR, NMI, and BEV
	or		t0,t1,t0		# Include Status.CEE (from UDI)
	mtc0	t0,_CP0_STATUS


#	===== Skip _on_bootstrap call =====
#
#	##################################################################
#	# Call the "on bootstrap" procedure
#	##################################################################
#	la	t0,_on_bootstrap
#	jalr	t0
#	nop


	##################################################################
	# DRN: Set up for vectored interrupts
	##################################################################
	# If the value of the exception base register is to be changed,
	# this must be done with StatusBEV equal ‘1’. The operation of the
	# processor is undefined if the Exception Base field is written with
	# a different value when StatusBEV is ‘0’. So...
	# Set BEV bit
	mfc0	t0,_CP0_STATUS		# get status
	or		t0,t0,0x00400000	# Set BEV (bootstrap exception mode)
	mtc0	t0,_CP0_STATUS		# set status
	# Initialize EBase register
	la	t1,_ebase_address
	mtc0	t1,_CP0_EBASE
	# Initialize IntCtl register
	la		t1,_vector_spacing
	li		t2,0				# Clear t2 and
	ins		t2,t1,5,5			# shift value to VS field
	mtc0	t2,_CP0_INTCTL
	# Note: "cause" register IV (use special interrupt vector) bit was
	# set above. If the CauseIV is 1 and StatusBEV is 0, the special
	# interrupt vector represents the base of the vectored interrupt table
	#
	# Clear BEV and EXL (exception level, force kernel mode) bits,
	# note ERL (error-level) is preserved.
	mfc0	t0,_CP0_STATUS
	and		t0,t0,0xffbffffD	# Clear BEV (vectored exception mode)
	# Note: original code used mask 0xffbffffD (didn't clear EXL bit)
	mtc0	t0,_CP0_STATUS
	# Interrupt controller: set Multi-Vectored bit
	xor		t0,t0,t0
	ori		t0,0x1000
	sw		t0,INTCONSET

	##################################################################
	# Branch to _start, which is CS3 C-environment initialization
	##################################################################
	//jal _start
        .global lowlevel_hw_init
        jal     lowlevel_hw_init
        nop

        .global _cinit
        jal     _cinit
        nop
        
        jal     __cs3_premain


#	===== Skip BEV and General Exception branch code for now =====
#
#	##################################################################
#	# Boot Exception Vector Handler
#	# Jumps to _bootstrap_exception_handler
#	##################################################################
#	.section .bev_handler,"ax",@progbits
#	.set noreorder
#	.ent _bev_exception
#_bev_exception:
#	la	k0,_bootstrap_exception_handler
#	jr	k0
#	nop
#	.end _bev_exception
#
#	##################################################################
#	# General Exception Vector Handler
#	# Jumps to _general_exception_handler
#	##################################################################
	.section .gen_handler,"ax",@progbits
	.set noreorder
	.ent _gen_exception
_gen_exception:
	la	k0,_general_exception_context
	j	k0
	nop
	.end _gen_exception

