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
 * \file nxp/stack_malloc.c
 * Overrides the stack malloc function to allocate from ethernet memory on the
 * NXP LPCxx68 processors.
 *
 * @author Balazs Racz
 * @date 23 June 2014
 */

#include <string.h>
#include <stdlib.h>
#include "utils/blinker.h"
#include "utils/constants.hxx"

/// Linker symbol defining the address where the ethernet RAM segment starts in
/// the 32-bit address space.
extern char __ETHRAM_segment_start__;
/// Next byte that's free in the stack memory allocation segment.
static char* sstack_start = &__ETHRAM_segment_start__;
/// Linker symbol defining the first byte that is used by the interrupt stack in
/// the stack segment.
extern char __stacks_min__;

void* usb_malloc(unsigned long length);

/// Set this to true to allocate stacks in a separate memory segment (usually
/// ethernet RAM). If false, stack will be allocated by malloc() on the heap.
DECLARE_CONST(use_separate_stack_segment);

/** Custom malloc function for stack spaces.
 *  \param length the length of block to allocate
 *  \returns a pointer to the newly allocated block.
 *
 *  There is currently no way to free blocks allocated with this function, so
 *  only suitable for stacks of threads that are running throughout the entire
 *  life of the application.
 */
void* stack_malloc(unsigned long length)
{
    if (config_use_separate_stack_segment() == CONSTANT_TRUE) {
        char* old_stack_start = sstack_start;
        char* new_stack_start = sstack_start + length;
        if (new_stack_start > &__stacks_min__)
        {
            diewith(BLINK_DIE_OUTOFMEMSTACK);
        }
        sstack_start = new_stack_start;
        return old_stack_start;
    } else {
        return usb_malloc(length);
    }
}

/// Linker symbol defining the address where the USB RAM segment starts in the
/// 32-bit address space.
extern char __USBRAM_segment_start__;
/// Linker symbol defining the address where the USB RAM segment ends in the
/// 32-bit address space.
extern char __USBRAM_segment_end__;
/// Pointer to the next free (unallocated) byte in the USB_RAM segment.
static char* ublock_start = &__USBRAM_segment_start__;

/** Custom malloc function for USB space.
 *  \param length the length of block to allocate
 *  \returns a pointer to the newly allocated block.
 *
 *  There is currently no way to free blocks allocated with this function, so
 *  only suitable for blocks that are running throughout the entire
 *  life of the application.
 */
void* usb_malloc(unsigned long length)
{
    // Aligns to 4 bytes.
    length += 3; length &= ~3;
    char* old_ublock_start = ublock_start;
    char* new_ublock_start = ublock_start + length;
    if (new_ublock_start > &__USBRAM_segment_end__)
    {
        diewith(BLINK_DIE_OUTOFMEMSTACK);
    }
    ublock_start = new_ublock_start;
    return old_ublock_start;
}

/// Allocates a buffer. Overrides the (weak) definition to put it to a
/// separate RAM segment and leave more heap space free.
/// @param size in bytes, how large chunk we should allocate.
/// @return a newly allocated buffer. Cannot be freed.
void *buffer_malloc(size_t length)
{
    /* We do a trick here to ensure that the compiler will output a stack frame
     * for this function. We want to avoid tail-chain optimization in this
     * function or else it disappears from the stack traces done for memory
     * tracing. */
    void *volatile v = usb_malloc(length);
    return v;
}
