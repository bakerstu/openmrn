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
 * \file Stm32EEPROMEmulation.hxx
 * This file implements STM32F0xx compatible EEPROM emulation in FLASH.
 *
 * @author Stuart W. Baker
 * @date 24 June 2015
 */

#ifndef _FREERTOS_DRIVERS_ST_STM32F0xxEEPROMEMULATION_HXX_
#define _FREERTOS_DRIVERS_ST_STM32F0xxEEPROMEMULATION_HXX_

#include "EEPROMEmulation.hxx"

/** Emulates EEPROM in FLASH for the STM32F0xx platform.  The EEPROM file size
 * is limited to the @ref SECTOR_SIZE / 2.  For the STM32, the erase size is
 * technically a "page" boundary, so the minimum value of what we call
 * SECTOR_SIZE in this driver is what ST calls a "page" size.
 */
class Stm32EEPROMEmulation : public EEPROMEmulation
{
public:
    /** Constructor.
     * @param name device name
     * @param file_size maximum file size that we can grow to.
     */
    Stm32EEPROMEmulation(const char *name, size_t file_size);

    /** Destructor.
     */
    ~Stm32EEPROMEmulation()
    {
    }

private:
    /** The erase page length for the particular MCU. SECTOR_SIZE must be a
     * multiple of this. */
    static const uintptr_t PAGE_SIZE;

    static inline const uint32_t* get_block(unsigned sector, unsigned offset);

    /**
     * Computes the pointer to load the data stored in a specific block from.
     * @param sector sector number [0..sectorCount_ - 1]
     * @param offset block index within sector, [0..rawBlockCount_ - 1]
     * @return pointer to the beginning of the data in the block. Must be alive until the next call to this function.
     */
    const uint32_t* block(unsigned sector, unsigned offset) override;

    /** Simple hardware abstraction for FLASH erase API.
     * @param sector Number of sector [0.. sectorCount_ - 1] to erase
     */
    void flash_erase(unsigned sector) override;

    /** Simple hardware abstraction for FLASH program API.
     * @param sector the sector to write to [0..sectorCount_ - 1]
     * @param start_block the block index to start writing to [0..rawBlockCount_ - 1]
     * @param data a pointer to the data to be programmed
     * @param byte_count the number of bytes to be programmed.
     *              Must be a multiple of BLOCK_SIZE
     */
    void flash_program(unsigned sector, unsigned start_block, uint32_t *data, uint32_t byte_count) override;

    /** Default constructor.
     */
    Stm32EEPROMEmulation();

    DISALLOW_COPY_AND_ASSIGN(Stm32EEPROMEmulation);
};

#endif /* _FREERTOS_DRIVERS_ST_STM32F0xxEEPROMEMULATION_HXX_ */
