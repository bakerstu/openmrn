/** @copyright
 * Copyright (c) 2018, Stuart W Baker
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
 * @file Stm32SPIFFS.cxx
 * This file implements a SPIFFS FLASH driver specific to CC32xx.
 *
 * @author Stuart W. Baker
 * @date 1 January 2018
 */

#include "Stm32SPIFFS.hxx"
#include "spiffs.h"
#include "stm32f_hal_conf.hxx"

//
// Stm32SPIFFS::flash_read()
//
int32_t Stm32SPIFFS::flash_read(uint32_t addr, uint32_t size, uint8_t *dst)
{
    HASSERT(addr >= fs_->cfg.phys_addr &&
            (addr + size) <= (fs_->cfg.phys_addr  + fs_->cfg.phys_size));

    memcpy(dst, (void*)addr, size);

    return 0;
}

//
// Stm32SPIFFS::flash_write()
//
int32_t Stm32SPIFFS::flash_write(uint32_t addr, uint32_t size, uint8_t *src)
{
    union WriteWord
    {
        uint8_t  data[8];
        uint16_t data_hword[4];
        uint32_t data_word[2];
        uint64_t data_dw;
    };

    HASSERT(addr >= fs_->cfg.phys_addr &&
            (addr + size) <= (fs_->cfg.phys_addr  + fs_->cfg.phys_size));

    HAL_FLASH_Unlock();

    if (addr & 1) {
        // Unaligned write at the beginning.
        WriteWord ww;
        ww.data_hword[0] = 0xffff;
        ww.data[1] = src[0];

        HASSERT(HAL_OK ==
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr - 1, ww.data_dw));
        addr += 1;
        size -= 1;
        src += 1;
    }

    while (size > 8) {
        WriteWord ww;
        memcpy(ww.data, src, 8);
        HASSERT(HAL_OK ==
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, ww.data_dw));
        addr += 8;
        size -= 8;
        src += 8;
    }

    while (size > 2) {
        WriteWord ww;
        memcpy(ww.data_hword, src, 2);
        HASSERT(HAL_OK ==
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, ww.data_dw));
        addr += 2;
        size -= 2;
        src += 2;
    }

    if (size) {
        // Unaligned write at the end of the range.
        HASSERT(size == 1);
        HASSERT((addr & 1) == 0);

        WriteWord ww;
        ww.data_hword[0] = 0xffff;
        ww.data[0] = src[0];

        HASSERT(HAL_OK ==
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, ww.data_dw));
        addr += 1;
        size -= 1;
        src += 1;
    }

    HAL_FLASH_Lock();
    
    return 0;
}

//
// Stm32SPIFFS::flash_erase()
//
int32_t Stm32SPIFFS::flash_erase(uint32_t addr, uint32_t size)
{
    HASSERT(addr >= fs_->cfg.phys_addr &&
            (addr + size) <= (fs_->cfg.phys_addr  + fs_->cfg.phys_size));
    HASSERT((size % ERASE_PAGE_SIZE) == 0);
    HASSERT((size % FLASH_PAGE_SIZE) == 0);

    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = (uint32_t)addr;
    erase_init.NbPages = size / FLASH_PAGE_SIZE;
    uint32_t page_error;
    
    HASSERT(HAL_OK == HAL_FLASHEx_Erase(&erase_init, &page_error));

    return 0;
}

