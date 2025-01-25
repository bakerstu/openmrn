/** \copyright
 * Copyright (c) 2015, Stuart W Baker
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
 * \file Stm32EEPROMEmulation.cxx
 * This file implements STM32F0xx compatible EEPROM emulation in FLASH.
 *
 * @author Stuart W. Baker
 * @date 24 June 2015
 */

#include "Stm32EEPROMEmulation.hxx"

#include <cstring>

#include "stm32f_hal_conf.hxx"

#if defined (STM32F030x6) || defined (STM32F031x6) || defined (STM32F038xx) \
 || defined (STM32F030x8) || defined (STM32F030xC) || defined (STM32F042x6) \
 || defined (STM32F048xx) || defined (STM32F051x8) || defined (STM32F058xx)
const size_t Stm32EEPROMEmulation::PAGE_SIZE = 0x400;
const size_t EEPROMEmulation::BLOCK_SIZE = 4;
const size_t EEPROMEmulation::BYTES_PER_BLOCK = 2;
#elif defined (STM32F070x6) || defined (STM32F070xB) || defined (STM32F071xB) \
   || defined (STM32F072xB) || defined (STM32F078xx) \
   || defined (STM32F091xC) || defined (STM32F098xx) \
   || defined (STM32F303xC) || defined (STM32F303xE)
const size_t Stm32EEPROMEmulation::PAGE_SIZE = 0x800;
const size_t EEPROMEmulation::BLOCK_SIZE = 4;
const size_t EEPROMEmulation::BYTES_PER_BLOCK = 2;
#elif defined(STM32L432xx) || defined(STM32L431xx)
const size_t Stm32EEPROMEmulation::PAGE_SIZE = 0x800;
const size_t EEPROMEmulation::BLOCK_SIZE = 8;
const size_t EEPROMEmulation::BYTES_PER_BLOCK = 4;
#define L4_FLASH
#elif defined(STM32F767xx)
// Note this assumes single-bank usage
const size_t Stm32EEPROMEmulation::PAGE_SIZE = 256*1024;
const size_t EEPROMEmulation::BLOCK_SIZE = 8;
const size_t EEPROMEmulation::BYTES_PER_BLOCK = 4;
#define F7_FLASH
#elif defined(STM32G0B1xx)
// Note this assumes single-bank usage
const size_t Stm32EEPROMEmulation::PAGE_SIZE = 0x800;
const size_t EEPROMEmulation::BLOCK_SIZE = 8;
const size_t EEPROMEmulation::BYTES_PER_BLOCK = 4;
#define L4_FLASH
#else
#error "stm32EEPROMEmulation unsupported STM32 device"
#endif

/** Constructor.
 * @param name device name
 * @param file_size maximum file size that we can grow to.
 */
Stm32EEPROMEmulation::Stm32EEPROMEmulation(const char *name, size_t file_size)
    : EEPROMEmulation(name, file_size)
{
    HASSERT(SECTOR_SIZE % PAGE_SIZE == 0);
    mount();
}

inline const uint32_t *Stm32EEPROMEmulation::get_block(
    unsigned sector, unsigned offset)
{
    return (uint32_t*)(&__eeprom_start + sector * EEPROMEmulation::SECTOR_SIZE + offset * EEPROMEmulation::BLOCK_SIZE);
}

/**
 * Computes the pointer to load the data stored in a specific block from.
 * @param sector sector number [0..sectorCount_ - 1]
 * @param offset block index within sector, [0..rawBlockCount_ - 1]
 * @return pointer to the beginning of the data in the block. Must be alive until the next call to this function.
 */
const uint32_t* Stm32EEPROMEmulation::block(unsigned sector, unsigned offset) {
    return get_block(sector, offset);
}

#ifdef F7_FLASH
extern "C" {
extern const unsigned __stm32_eeprom_flash_sector_start;
}
#endif

/** Simple hardware abstraction for FLASH erase API.
 * @param sector Number of sector [0.. sectorCount_ - 1] to erase
 */
void Stm32EEPROMEmulation::flash_erase(unsigned sector)
{
    HASSERT(sector < sectorCount_);
    uint32_t page_error;

#ifdef F7_FLASH
    FLASH_EraseInitTypeDef erase_init;
    memset(&erase_init, 0, sizeof(erase_init));

    erase_init.TypeErase = TYPEERASE_SECTORS;
    erase_init.Sector =
        ((unsigned)(&__stm32_eeprom_flash_sector_start)) + sector;
    erase_init.NbSectors = 1;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 3.3 to 3.6 volts powered.
    HASSERT(SECTOR_SIZE == PAGE_SIZE);
    portENTER_CRITICAL();
    HAL_FLASH_Unlock();
    HASSERT(HAL_OK == HAL_FLASHEx_Erase(&erase_init, &page_error));
    HAL_FLASH_Lock();
    portEXIT_CRITICAL();
    
#else    
    auto* address = get_block(sector, 0);
    
    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
#ifdef L4_FLASH
    erase_init.Banks = FLASH_BANK_1;
    uint32_t start_page = erase_init.Page =
        (((uint32_t)address) - FLASH_BASE) / PAGE_SIZE;
#else    
    erase_init.PageAddress = (uint32_t)address;
#endif    
    erase_init.NbPages = SECTOR_SIZE / PAGE_SIZE;

    portENTER_CRITICAL();
    HAL_FLASH_Unlock();
    // We erase the first page at the end, because the magic bytes are
    // there. This is to make corruption less likely in case of a power
    // interruption happens.
    if (SECTOR_SIZE > PAGE_SIZE) {
#ifdef L4_FLASH
        erase_init.Page += 1;
#else        
        erase_init.PageAddress += PAGE_SIZE;
#endif        
        erase_init.NbPages--;
        HAL_FLASHEx_Erase(&erase_init, &page_error);
        erase_init.NbPages = 1;
#ifdef L4_FLASH
        erase_init.Page = start_page;
#else
        erase_init.PageAddress = (uint32_t)address;
#endif        
    }
    HAL_FLASHEx_Erase(&erase_init, &page_error);
    HAL_FLASH_Lock();
    portEXIT_CRITICAL();
#endif    
}

/** Simple hardware abstraction for FLASH program API.
 * @param sector the sector to write to [0..sectorCount_ - 1]
 * @param start_block the block index to start writing to [0..rawBlockCount_ -
 * 1]
 * @param data a pointer to the data to be programmed
 * @param count the number of bytes to be programmed.
 *              Must be a multiple of BLOCK_SIZE
 */
void Stm32EEPROMEmulation::flash_program(
    unsigned relative_sector, unsigned start_block, uint32_t *data, uint32_t count)
{
    HASSERT(relative_sector < sectorCount_);
    HASSERT((count % BLOCK_SIZE) == 0);
    HASSERT(start_block + (count / BLOCK_SIZE) <= rawBlockCount_);
    auto* address = get_block(relative_sector, start_block);

    uintptr_t uint_address = (uintptr_t)address;

    /* The STM32 program size is 16-bits, however BLOCK_SIZE is [at least]
     * 32-bits.  Because the upper 16-bits contains the EEPROM address, while
     * the lower 16-bits contains the data.  0xFFFF is not considered a valid
     * address by the generic EEPROM driver, therefore, so long as the data
     * half-word is written ahead of the address half-word, there is no race
     * condition on power failure.
     */
    while (count)
    {
        portENTER_CRITICAL();
        HAL_FLASH_Unlock();
#if defined(F7_FLASH)|| defined(L4_FLASH)
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, uint_address, *(uint64_t*)data);
#else        
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, uint_address, *data);
#endif        
        HAL_FLASH_Lock();
        portEXIT_CRITICAL();

        count -= BLOCK_SIZE;
        uint_address += BLOCK_SIZE;
        data += (BLOCK_SIZE / sizeof(uint32_t));
    }
}
