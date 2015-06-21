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

#include "EEPROM.hxx"

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/flash.h"


/** Emulates EEPROM in FLASH for the Tiva platform.
 * @todo there is a known bug whereby the ADDRESS_SPACE cannot be larger than
 *       (block size - 4).
 */
class TivaEEPROMEmulation : public EEPROM
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
     * @param file_size maximum file size that we can grow to.
     */
    TivaEEPROMEmulation(const char *name, size_t file_size);

    /** Destructor.
     */
    ~TivaEEPROMEmulation();

    /** Write to the EEPROM.  NOTE!!! This is not necessarily atomic across
     * byte boundaries in the case of power loss.  The user should take this
     * into account as it relates to data integrity of a whole block.
     * @ref index index within EEPROM address space to start write
     * @ref buf data to write
     * @ref len length in bytes of data to write
     */
    void write(unsigned int index, const void *buf, size_t len) OVERRIDE;

    /** Read from the EEPROM.
     * @ref index index within EEPROM address space to start read
     * @ref buf location to post read data
     * @ref len length in bytes of data to read
     */
    void read(unsigned int index, void *buf, size_t len) OVERRIDE;

private:
    /** Total FLASH memory size to use for EEPROM Emulation.  Must be at least
     * 2 block large and at least 4x the total amount of EEPROM address space
     * that will be emulated.  Larger sizes will result in greater endurance.
     */
    static const size_t FLASH_SIZE;

    /** @ref Family that device belongs to */
    static const Family FAMILY;

    /** Address space in terms of bytes to emulate, must be less than 2^16. */
    static const size_t ADDRESS_SPACE;

    /** Shadow the EEPROM data in RAM.  This will increase read performance at
     * the expense of additional RAM usage.
     */
    static const bool SHADOW_IN_RAM;

    /** magic marker for an intact block */
    static const uint32_t MAGIC_INTACT;

    /** magic marker for a block that we are transitioning to intact */
    static const uint32_t MAGIC_DIRTY;

    /** magic marker for a used block */
    static const uint32_t MAGIC_USED;

    /** magic marker for an erased block */
    static const uint32_t MAGIC_ERASED;

    /** Raw FLASH for storing EEPROM emulated data. */
    static const uint16_t* const raw;

    /** Write to the EEPROM on a native word boundary.
     * @ref index word within EEPROM address space to write
     * @ref data data to write
     */
    void write_word(unsigned int index, const uint16_t data);

    /** Read from the EEPROM on a native word boundary.
     * @ref index word within EEPROM address space to read
     * @ref data location to place read data
     */
    void read_word(unsigned int index, uint16_t *data);

    /** Get the next active block pointer.
     * @return a pointer to the beginning of the next active block
     */
    uint32_t *next_active()
    {
        return ((activeIndex + 1) < block_count()) ? block(activeIndex + 1) :
                                                     block(0);
    }

    /** Get the active block pointer.
     * @return a pointer to the beginning of the active block
     */
    uint32_t *active()
    {
        return block(activeIndex);
    }

    /** Get the block index data pointer.
     * @param i index of block to get a data pointer to
     * @return a pointer to the beginning of the block index data
     */
    uint32_t *block(int i)
    {
        return (uint32_t*)(raw + ((FAMILY >> 1) * i));
    }

    /** Get the block index.
     * @param i index of block to get a pointer to
     * @return a pointer to the beginning of the block index
     */
    int block_index(uint32_t *block_address)
    {
        return ((uintptr_t)block_address - (uintptr_t)raw) / FAMILY;
    }

    /** Total number of FLASH blocks being used for emulation.
     * @return number of FLASH blocks being used for emulation
     */
    int block_count()
    {
        return FLASH_SIZE / FAMILY;
    }

    /** Total number of EEPROM slots in a FLASH block.
     * @return number of EEPROM slots in a FLASH block.
     */
    int slot_count()
    {
        return (FAMILY / sizeof(uint32_t)) - 1;
    }

    /** Simple TivaWare abstraction for FlashErase() API.
     * @param address the start address of the flash block to be erased
     */
    void flash_erase(void *address)
    {
        HASSERT(((uintptr_t)address % FAMILY) == 0);
        HASSERT((uintptr_t)address >= (uintptr_t)raw);
        HASSERT((uintptr_t)address < (uintptr_t)(raw + (FLASH_SIZE >> 1)));

        MAP_FlashErase((uint32_t)address);
    }

    /** Simple TivaWare abstraction for FlashProgram() API.
     * @param data a pointer to the data to be programmed
     * @param address the starting address in flash to be programmed.
     *                Must be a multiple of four.
     * @param count the number of bytes to be programmed.
     *              Must be a multiple of four
     */
    void flash_program(uint32_t *data, void *address, uint32_t count)
    {
        HASSERT(((uintptr_t)address % 4) == 0);
        HASSERT((uintptr_t)address >= (uintptr_t)raw);
        HASSERT((uintptr_t)address < (uintptr_t)(raw + (FLASH_SIZE >> 1)));
        HASSERT((count % 4) == 0);

        MAP_FlashProgram(data, (uint32_t)address, count);
    }

    /** pointer to RAM for shadowing EEPROM. */
    uint16_t *shadow;

    /** index of the active block */
    int activeIndex;

    /** number of available slots for new data */
    size_t available;

    /** Default constructor.
     */
    TivaEEPROMEmulation();

    DISALLOW_COPY_AND_ASSIGN(TivaEEPROMEmulation);
};

#endif /* _FREERTOS_DRIVERS_TI_TIVAEEPROMEMULATION_HXX_ */
