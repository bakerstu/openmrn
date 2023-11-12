/** @copyright
 * Copyright (c) 2023, Balazs Racz
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
 * @file Stm32Flash.hxx
 *
 * This file implements a generic Flash driver specific to STM32 F7 flash
 * API.
 *
 * @author Balazs Racz
 * @date 4 Sep 2023
 */

#include "freertos_drivers/st/Stm32Flash.hxx"

uint32_t STM32F7_DUAL_BANK_2M_FLASH[] =
{
    0x08000000,
    0x08004000,
    0x08008000,
    0x0800C000,
    0x08010000,
    0x08020000,
    0x08040000,
    0x08060000,
    0x08080000,
    0x080A0000,
    0x080C0000,
    0x080E0000,
    0x08100000,
    0x08104000,
    0x08108000,
    0x0810C000,
    0x08110000,
    0x08120000,
    0x08140000,
    0x08160000,
    0x08180000,
    0x081A0000,
    0x081C0000,
    0x081E0000,
    FLASH_EOF
};

static_assert(ARRAYSIZE(STM32F7_DUAL_BANK_2M_FLASH) == 25,
              "dual bank 2M flash mismatch");

uint32_t STM32F7_SINGLE_BANK_2M_FLASH[] =
{
    0x08000000,
    0x08008000,
    0x08010000,
    0x08018000,
    0x08020000,
    0x08040000,
    0x08080000,
    0x080C0000,
    0x08100000,
    0x08140000,
    0x08180000,
    0x081C0000,
    FLASH_EOF
};

static_assert(ARRAYSIZE(STM32F7_SINGLE_BANK_2M_FLASH) == 13,
              "single bank 2M flash mismatch");
