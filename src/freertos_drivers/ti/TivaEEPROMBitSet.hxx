/** \copyright
 * Copyright (c) 2017, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file TivaEEPROMBitSet.hxx
 *
 * Implementation for persistent bit set that uses Tiva EEPROM as backing
 * storage.
 *
 * @author Balazs Racz
 * @date 7 June 2017
 */

#ifndef _FREERTOS_DRIVER_TI_TIVAEEPROMBITSET_HXX_
#define _FREERTOS_DRIVER_TI_TIVAEEPROMBITSET_HXX_

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include "driverlib/eeprom.h"
#include "driverlib/sysctl.h"

#include "utils/EEPROMStoredBitSet.hxx"

template <unsigned USER_BIT_COUNT, unsigned BITS_PER_CELL = 27>
class TivaEEPROMHwDefs : public EEPROMStoredBitSet_DefaultHW
{
protected:
    /// For the Tiva 123 use block_count as multiple of 2, for Tiva 129 use
    /// multiple of 8. Same with block_start.
    TivaEEPROMHwDefs(uint8_t block_start, uint8_t block_count)
        : blockStart_(block_start)
        , blockCount_(block_count)
    {
        HASSERT((block_start % block_count) == 0);
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
        do
        {
            auto r = MAP_EEPROMInit();
            if (r == EEPROM_INIT_OK)
                break;
            MAP_SysCtlDelay(100000);
        } while(true);
        auto num_blocks = MAP_EEPROMBlockCountGet();
        HASSERT(block_start + block_count < num_blocks);
        uint32_t total_size = MAP_EEPROMSizeGet();
        // The hardware for which this driver is written has 16 words per
        // block.
        HASSERT(total_size / num_blocks == 64);
    }

    /// @return how many user bits we store per physical cell.
    static constexpr unsigned bits_per_cell()
    {
        return BITS_PER_CELL;
    }

    static constexpr unsigned virtual_cell_count()
    {
        return (USER_BIT_COUNT + bits_per_cell()) / bits_per_cell();
    }

    unsigned physical_cell_count()
    {
        return blockCount_ * 16; // Tiva has 16 cells per block (64 bytes)
    }

    void write_cell(unsigned cell_offset, eeprom_t value)
    {
        MAP_EEPROMProgram(&value, get_address(cell_offset), 4);
    }

    eeprom_t read_cell(unsigned cell_offset)
    {
        eeprom_t ret;
        MAP_EEPROMRead(&ret, get_address(cell_offset), 4);
        return ret;
    }

private:
    /// @return the address for TivaWare of the given physical cell.
    /// @param cell_offset is the physical cell number.
    uint32_t get_address(unsigned cell_offset)
    {
        uint32_t address = blockStart_;
        address = EEPROMAddrFromBlock(address);
        address += (cell_offset << 2);
        return address;
    }

    /// Number of the first block for our data.
    uint8_t blockStart_;
    /// Number of the blocks of 16 words each to use for our data.
    uint8_t blockCount_;
};

#endif // _FREERTOS_DRIVER_TI_TIVAEEPROMBITSET_HXX_
