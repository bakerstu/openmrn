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
 * \file stack_malloc.c
 * Overrides the stack malloc function to allocate from ethernet memory on the
 * NXP LPCxx68 processors.
 *
 * @author Balazs Racz
 * @date 23 June 2014
 */

#include <string.h>
#include <stdlib.h>
#include "utils/blinker.h"

extern char __ETHRAM_segment_start__;
//static char* sstack_start = &__ETHRAM_segment_start__;
extern char __stacks_min__;

void* usb_malloc(unsigned long length);


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
    return usb_malloc(length);
/*    char* old_stack_start = sstack_start;
    char* new_stack_start = sstack_start + length;
    if (new_stack_start > &__stacks_min__)
    {
        diewith(BLINK_DIE_OUTOFMEMSTACK);
    }
    sstack_start = new_stack_start;
    return old_stack_start;*/
}

extern char __USBRAM_segment_start__;
extern char __USBRAM_segment_end__;
//static char* ublock_start = &__USBRAM_segment_start__;

extern char __bss_end__;
static char* ublock_start = &__bss_end__;

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

void *buffer_malloc(size_t size)
{
    /* We do a trick here to ensure that the compiler will output a stack frame
     * for this function. We want to avoid tail-chain optimization in this
     * function or else it disappears from the stack traces done for memory
     * tracing. */
    void *volatile v = stack_malloc(size);
    return v;
}

struct _reent* allocate_reent(void)
{
    struct _reent* data = usb_malloc(sizeof(struct _reent));
    _REENT_INIT_PTR(data);
    return data;
}
