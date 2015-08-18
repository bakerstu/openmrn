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
    /** @ref Family that device belongs to */
    static const unsigned FAMILY;

    /** Lookup sector number from address.
     * @param address sector address;
     * @return sector number.
     */
    int address_to_sector(const void *address) OVERRIDE;

    /** Lookup address number from sector number.
     * @param sector sector number;
     * @return sector address.
     */
    uint32_t *sector_to_address(const int sector) OVERRIDE;

    /** Simple hardware abstraction for FLASH erase API.
     * @param address the start address of the flash block to be erased
     */
    void flash_erase(void *address);

    /** Simple hardware abstraction for FLASH program API.
     * @param data a pointer to the data to be programmed
     * @param address the starting address in flash to be programmed.
     *                Must be a multiple of BLOCK_SIZE
     * @param count the number of bytes to be programmed.
     *              Must be a multiple of BLOCK_SIZE
     */
    void flash_program(uint32_t *data, void *address, uint32_t count) OVERRIDE;

    /** Default constructor.
     */
    TivaEEPROMEmulation();

    DISALLOW_COPY_AND_ASSIGN(TivaEEPROMEmulation);
};

#endif /* _FREERTOS_DRIVERS_TI_TIVAEEPROMEMULATION_HXX_ */
