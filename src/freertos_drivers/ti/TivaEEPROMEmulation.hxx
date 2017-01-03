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
 * \file TivaEEPROMEmulation.hxx
 * This file implements Tiva compatible EEPROM emulation in FLASH.
 *
 * @author Stuart W. Baker
 * @date 21 January 2015
 */

#ifndef _FREERTOS_DRIVERS_TI_TIVAEEPROMEMULATION_HXX_
#define _FREERTOS_DRIVERS_TI_TIVAEEPROMEMULATION_HXX_

#include "EEPROMEmulation.hxx"

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/flash.h"


/** Emulates EEPROM in FLASH for the Tiva platform.
 * @todo there is a known bug whereby the ADDRESS_SPACE cannot be larger than
 *       (block size - 4).
 */
class TivaEEPROMEmulation : public EEPROMEmulation
{
public:
    /** Product family.  We use this to determine details, such as sector size,
     * specific to a given family and relevant to the agorithm.
     */
    enum Family
    {
        TM4C123 = 1024, /**< Tiva TM4C123 devices, 1K block size */
        TM4C129 = 1024 * 16 /**< Tiva TM4C129 devices, 16K block Size */
    };

    /** Constructor.
     * @param name device name
     * @param file_size_bytes maximum file size that we can grow to.
     */
    TivaEEPROMEmulation(const char *name, size_t file_size_bytes);

    /** Destructor.
     */
    ~TivaEEPROMEmulation()
    {
    }

private:
    static inline const uint32_t* get_block(unsigned sector, unsigned offset);
    
    /** @ref Family that device belongs to */
    static const unsigned FAMILY;

    /** Simple hardware abstraction for FLASH erase API.
     * @param address the start address of the flash block to be erased
     */
    void flash_erase(unsigned sector) override;

    /** Simple hardware abstraction for FLASH program API.
     * @param sector the sector to write to
     * @param start_block the block index to start writing to
     * @param data a pointer to the data to be programmed
     * @param byte_count the number of bytes to be programmed.
     *              Must be a multiple of BLOCK_SIZE
     */
    void flash_program(unsigned sector, unsigned start_block, uint32_t *data,
        uint32_t byte_count) override;

    /**
     * Computes the pointer to load the data stored in a specific block from.
     * @param sector sector number [0..sectorCount_ - 1]
     * @param offset block index within sector, [0..rawBlockCount_ - 1]
     * @return pointer to the beginning of the data in the block. Must be alive until the next call to this function.
     */
    const uint32_t* block(unsigned sector, unsigned offset) override;
    
    /** Default constructor.
     */
    TivaEEPROMEmulation();

    DISALLOW_COPY_AND_ASSIGN(TivaEEPROMEmulation);
};

#endif /* _FREERTOS_DRIVERS_TI_TIVAEEPROMEMULATION_HXX_ */
