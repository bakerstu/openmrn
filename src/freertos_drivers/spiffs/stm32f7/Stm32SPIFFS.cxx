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

    flash_.read(addr, size, dst);

    return 0;
}

//
// Stm32SPIFFS::flash_write()
//
int32_t Stm32SPIFFS::flash_write(uint32_t addr, uint32_t size, uint8_t *src)
{
    HASSERT(addr >= fs_->cfg.phys_addr &&
            (addr + size) <= (fs_->cfg.phys_addr  + fs_->cfg.phys_size));

    flash_.write(addr, size, src);

    return 0;
}

//
// Stm32SPIFFS::flash_erase()
//
int32_t Stm32SPIFFS::flash_erase(uint32_t addr, uint32_t size)
{
    HASSERT(addr >= fs_->cfg.phys_addr &&
            (addr + size) <= (fs_->cfg.phys_addr  + fs_->cfg.phys_size));

    flash_.erase(addr, size);

/*
  This is the old flash erase code. This will be needed in Stm32Flash when we want to support the F3, F0 or L4 MCUs.

  
    extern char __flash_fs_start;
    extern char __flash_fs_sector_start;

    FLASH_EraseInitTypeDef erase_init;
    memset(&erase_init, 0, sizeof(erase_init));

    erase_init.TypeErase = TYPEERASE_SECTORS;
    unsigned offset = addr - (unsigned)&__flash_fs_start;
    HASSERT((size % ERASE_PAGE_SIZE) == 0);
    HASSERT((offset % ERASE_PAGE_SIZE) == 0);
    unsigned sector = offset / ERASE_PAGE_SIZE;
    erase_init.Sector = ((unsigned)(&__flash_fs_sector_start)) + sector;
    erase_init.NbSectors = size / ERASE_PAGE_SIZE;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 3.3 to 3.6 volts powered.
    HAL_FLASH_Unlock();
    uint32_t sector_error;
    HASSERT(HAL_OK == HAL_FLASHEx_Erase(&erase_init, &sector_error));
    HAL_FLASH_Lock();
*/

    return 0;
}

