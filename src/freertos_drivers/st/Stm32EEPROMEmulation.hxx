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

/** Emulates EEPROM in FLASH for the STM32F0xx platform.
 * The EEPROM file size is limited to the @ref SECTOR_SIZE / 2.  For the STM32,
 * the erase size is technically a "page" boundary, so what we call SECTOR_SIZE
 * in this driver really refers to what ST calls a "page" size.
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
    /** write size in bytes, must be macro to use as an array size */
    #define WRITE_SIZE 256

    /** Start address of FLASH */
    static const uintptr_t FLASH_START;

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
    void flash_erase(void *address) override;

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
    Stm32EEPROMEmulation();

    DISALLOW_COPY_AND_ASSIGN(Stm32EEPROMEmulation);
};

#endif /* _FREERTOS_DRIVERS_ST_STM32F0xxEEPROMEMULATION_HXX_ */
