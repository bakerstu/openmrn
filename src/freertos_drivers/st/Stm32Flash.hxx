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

#include <algorithm>

#include "utils/logging.h"

#include "stm32f_hal_conf.hxx"

/// Strategy module for finding flash erase sector numbers when the sectors are
/// constant size.
template <uint32_t ERASE_PAGE_SIZE> struct FlashFixedSectors
{
    /// Aligns an address to the next possible sector start (i.e., rounds up to
    /// sector boundary).
    /// @param addr an address in the flash address space.
    /// @return If addr is the first byte of a sector, then returns addr
    /// unmodified. Otherwise returns the starting address of the next sector.
    static uint32_t next_sector_address(uint32_t addr)
    {
        static_assert(
            ((ERASE_PAGE_SIZE - 1) << 1) & ERASE_PAGE_SIZE == ERASE_PAGE_SIZE,
            "Erase page size must be a power of two.");
        return (addr + ERASE_PAGE_SIZE - 1) & ~(ERASE_PAGE_SIZE - 1);
    }

    /// Lookup the next sector for an address, and return the {sector number,
    /// address} pair.
    static std::pair<unsigned, uint32_t> lookup_sector(uint32_t addr)
    {
        uint32_t next = next_sector_address(addr);
        unsigned sector = (next - FLASH_BASE) / ERASE_PAGE_SIZE;
        return {sector, next};
    }
};

static constexpr uint32_t FLASH_EOF = 0xFFFFFFFFul;

/// Strategy module for finding flash erase sector numbers when the sectors are
/// different sizes.
struct FlashVariableSectors
{
    /// bank_config is an array of uint32 addresses, containing the start of
    /// each sector. The sector number is the index in this array. As a
    /// sentinel, the element after the last shall be 0xfffffffful.
    constexpr FlashVariableSectors(uint32_t *bank_config)
        : bankConfig_(bank_config)
    { }

    /// Aligns an address to the next possible sector start (i.e., rounds up to
    /// sector boundary).
    /// @param addr an address in the flash address space.
    /// @return If addr is the first byte of a sector, then returns addr
    /// unmodified. Otherwise returns the starting address of the next sector.
    uint32_t next_sector_address(uint32_t addr)
    {
        if (addr == bankConfig_[0])
        {
            lastIndex_ = 0;
            return addr;
        }
        if (bankConfig_[lastIndex_] >= addr)
        {
            lastIndex_ = 0;
        }
        HASSERT(bankConfig_[lastIndex_] <= addr);
        while (bankConfig_[lastIndex_ + 1] < addr)
        {
            ++lastIndex_;
        }
        // now: bankConfig_[lastIndex_] < addr and
        // bankConfig_[lastIndex_+1] >= addr
        return bankConfig_[lastIndex_ + 1];
    }

    /// Lookup the next sector for an address, and return the {sector number,
    /// address} pair.
    std::pair<unsigned, uint32_t> lookup_sector(uint32_t addr)
    {
        uint32_t next = next_sector_address(addr);
        return {lastIndex_ + 1, next};
    }

protected:
    /// 1-element cache on where to start looking for
    /// sectors. bankConfig_[lastIndex_] <= last address that was queried.
    unsigned lastIndex_ {0};

private:
    /// Contains an array of the start addresses of the erase sectors.
    uint32_t *bankConfig_;
};

extern uint32_t STM32F7_DUAL_BANK_2M_FLASH[];
extern uint32_t STM32F7_SINGLE_BANK_2M_FLASH[];

template <class SectorLookup> class Stm32Flash : public SectorLookup {
public:
    template <typename... Args>
    constexpr Stm32Flash(Args &&...args)
        : SectorLookup(std::forward<Args>(args)...)
    { }

void read(uint32_t addr, uint32_t size, uint8_t *dst)
{
    memcpy(dst, (void *)addr, size);
}

void write(uint32_t addr, uint32_t size, uint8_t *src)
{
    union WriteWord
    {
        uint8_t  data[4];
        uint32_t data_word;
    };

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
        return;
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
        uint8_t *flash = (uint8_t *)addr;
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
}

void erase(uint32_t addr, uint32_t size)
{
    FLASH_EraseInitTypeDef erase_init;
    memset(&erase_init, 0, sizeof(erase_init));

    erase_init.TypeErase = TYPEERASE_SECTORS;

    auto sa = this->lookup_sector(addr);
    HASSERT(sa.second == addr); // start of erase must fall on sector boundary.
    erase_init.Sector = sa.first;

    unsigned count = 0;
    // Figure out how many total sectors we need to erase.
    do {
        ++count;
        sa = this->lookup_sector(sa.second + 1);
    } while (sa.second < addr + size);

    HASSERT(sa.second == addr + size || sa.second == FLASH_EOF);
    
    erase_init.NbSectors = count;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 3.3 to 3.6 volts powered.
    HAL_FLASH_Unlock();
    uint32_t sector_error;
    HASSERT(HAL_OK == HAL_FLASHEx_Erase(&erase_init, &sector_error));
    HAL_FLASH_Lock();
}

}; // class Stm32Flash
