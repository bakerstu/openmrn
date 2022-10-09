/** \copyright
 * Copyright (c) 2022, Balazs Racz
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
 * \file OSInterrupt.cxx
 *
 * ARM implementation of the cross-platform representation of an os-compatible
 * interrupt.
 *
 * @author Balazs Racz
 * @date 5 Oct 2022
 */

#include "os/OSInterrupt.hxx"

#include "freertos_includes.h"

static constexpr uint32_t NVIC_BASE = (0xE000E100UL);

static constexpr uint32_t NVIC_SET_ENABLE = NVIC_BASE + 0;
static constexpr uint32_t NVIC_CLEAR_ENABLE = NVIC_BASE + 0x80;
static constexpr uint32_t NVIC_SET_PENDING = NVIC_BASE + 0x100;
static constexpr uint32_t NVIC_CLEAR_PENDING = NVIC_BASE + 0x180;

static constexpr uint32_t NVIC_PRIO = NVIC_BASE + 0x300;

void OSInterrupt::initialize() const
{
    ((uint8_t*)NVIC_PRIO)[num_] = 0xFF;
#ifdef GCC_CM3
#elif GCC_CM0
#else
    #error dont know what cortex version we are compiling for
#endif    
}

void OSInterrupt::enable() const
{
    ((uint32_t*)NVIC_SET_ENABLE)[num_>>5] = (1u << (num_ & 31));
}

void OSInterrupt::disable() const
{
    ((uint32_t*)NVIC_CLEAR_ENABLE)[num_>>5] = (1u << (num_ & 31));
}

void OSInterrupt::set_pending() const
{
    ((uint32_t*)NVIC_SET_PENDING)[num_>>5] = (1u << (num_ & 31));
}

void OSInterrupt::clear_pending() const
{
    ((uint32_t*)NVIC_CLEAR_PENDING)[num_>>5] = (1u << (num_ & 31));
}
