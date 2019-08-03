/** @copyright
 * Copyright (c) 2019, Balazs Racz
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
 *
 * This file implements a SPIFFS FLASH driver specific to STM32 F7 flash
 * API.
 *
 * @author Balazs Racz
 * @date 13 July 2019
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
        uint8_t  data[4];
        uint32_t data_word;
    };

    HASSERT(addr >= fs_->cfg.phys_addr &&
            (addr + size) <= (fs_->cfg.phys_addr  + fs_->cfg.phys_size));

    HAL_FLASH_Unlock();

    if ((addr % 4) && ((addr % 4) + size) < 4)
    {
        // single unaligned write in the middle of a word.
        WriteWord ww;
        ww.data_word = 0xFFFFFFFF;

        memcpy(ww.data + (addr % 4), src, size);
        ww.data_word &= *((uint32_t*)(addr & (~0x3)));
        HASSERT(HAL_OK ==
            HAL_FLASH_Program(
                FLASH_TYPEPROGRAM_WORD, addr & (~0x3), ww.data_word));

        HAL_FLASH_Lock();
        return 0;
    }

    int misaligned = (addr + size) % 4;
    if (misaligned != 0)
    {
        // last write unaligned data
        WriteWord ww;
        ww.data_word = 0xFFFFFFFF;

        memcpy(&ww.data_word, src + size - misaligned, misaligned);
        ww.data_word &= *((uint32_t*)((addr + size) & (~0x3)));
        HASSERT(HAL_OK ==
            HAL_FLASH_Program(
                FLASH_TYPEPROGRAM_WORD, (addr + size) & (~0x3), ww.data_word));

        size -= misaligned;
    }

    misaligned = addr % 4;
    if (size && misaligned != 0)
    {
        // first write unaligned data
        WriteWord ww;
        ww.data_word = 0xFFFFFFFF;

        memcpy(ww.data + misaligned, src, 4 - misaligned);
        ww.data_word &= *((uint32_t*)(addr & (~0x3)));
        HASSERT(HAL_OK ==
            HAL_FLASH_Program(
                FLASH_TYPEPROGRAM_WORD, addr & (~0x3), ww.data_word));
        addr += 4 - misaligned;
        size -= 4 - misaligned;
        src  += 4 - misaligned;
    }

    HASSERT((addr % 4) == 0);
    HASSERT((size % 4) == 0);

    if (size)
    {
        // the rest of the aligned data
        uint8_t *flash = (uint8_t*)addr;
        for (uint32_t i = 0; i < size; i += 4)
        {
            src[i + 0] &= flash[i + 0];
            src[i + 1] &= flash[i + 1];
            src[i + 2] &= flash[i + 2];
            src[i + 3] &= flash[i + 3];
            HASSERT(HAL_OK ==
                HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i,
                    *(unsigned long *)(src + i)));
        }
    }

    HAL_FLASH_Lock();
    
    return 0;
}

//
// Stm32SPIFFS::flash_erase()
//
int32_t Stm32SPIFFS::flash_erase(uint32_t addr, uint32_t size)
{
    extern char __flash_fs_start;
    extern char __flash_fs_sector_start;
    HASSERT(addr >= fs_->cfg.phys_addr &&
            (addr + size) <= (fs_->cfg.phys_addr  + fs_->cfg.phys_size));

    FLASH_EraseInitTypeDef erase_init;
    memset(&erase_init, 0, sizeof(erase_init));

    erase_init.TypeErase = TYPEERASE_SECTORS;
    unsigned offset = addr - (unsigned) &__flash_fs_start;
    HASSERT((size % ERASE_PAGE_SIZE) == 0);
    HASSERT((offset % ERASE_PAGE_SIZE) == 0);
    unsigned sector = offset / ERASE_PAGE_SIZE;
    erase_init.Sector = 
        ((unsigned)(&__flash_fs_sector_start)) + sector;
    erase_init.NbSectors = size / ERASE_PAGE_SIZE;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 3.3 to 3.6 volts powered.
    HAL_FLASH_Unlock();
    uint32_t sector_error;
    HASSERT(HAL_OK == HAL_FLASHEx_Erase(&erase_init, &sector_error));
    HAL_FLASH_Lock();

    return 0;
}

