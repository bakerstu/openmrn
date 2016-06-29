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
 * \file EEPROMEmulation.hxx
 * This file implements a generic EEPROM emulation in FLASH.
 *
 * @author Stuart W. Baker
 * @date 24 June 2015
 */

#ifndef _FREERTOS_DRIVERS_COMMON_EEPROMEMULATION_HXX_
#define _FREERTOS_DRIVERS_COMMON_EEPROMEMULATION_HXX_

#include "EEPROM.hxx"

/// Linker-defined symbol where in the memory space (flash) the eeprom
/// emulation data starts.
extern const char __eeprom_start;
/// Linker-defined symbol where in the memory space (flash) the eeprom
/// emulation data ends.
extern const char __eeprom_end;

/** Emulates EEPROM in FLASH for the Tiva, LPC17xx and LPC40xx
 * platforms. Applicable in general to any microcontroller with self-writeable
 * flash.
 *
 * Theory of operation:
 *
 * The EEPROM area is an area of the MCU Flash reserved for the EEPROMEmulation
 * driver. This area must fall onto flash erase boundaries. The driver will
 * perform a journal of every write into this flash area by writing (offset,
 * data) pairs in append mode. Reads will scan through the journal to find the
 * desired data. When the journal gets full, the data is copied to a second
 * flash area, compacting data by keeping overwrites only once, and then the
 * original area is erased.
 *
 * Specifics and parameters:
 *
 * The flash area is specified by the linker symbols __eeprom_start and
 * __eeprom_end. It is the responsibility of the memory map linker script to
 * align these at flash erase boundaries. Inside this area there are
 * independently eraseable sectors. The requirement is that at least two
 * independently eraseable sectors be present, in order to allow copying data
 * from one sector to another without endangering data loss due to power
 * interruption.
 *
 * The layout of each sector is the following: the sector is split into blocks,
 * where each block can be independently written. Each block will be written
 * only once between two erase operations. The first few blocks are reserved
 * for tracking the state of the sector, the rest of the blocks are used as
 * slots holding data payload. The sectors go through the following states (in
 * order):
 *  1) erased. When the sector is all 0xFF.
 *  2) dirty. The data is being copied over into this sector.
 *  3) intact. This sector contains all the data.
 *  4) used. This sector contains old data and can be reused after erasing.
 *
 * The layout of a slot is very simple: the first two bytes hold the
 * address. The lower two bytes of each 4-byte hold the data payload.
 *
 * Parameters:
 *  @param SECTOR_SIZE: size of independently erased flash areas. Usually in
 *  the range of kilobytes; for example somewhere between 1-16 kbytes.
 *  @param FLASH_SIZE: Automatically detected from the linker symbols. An
 *  integer (at least 2) multiple of SECTOR_SIZE. Sectors within the
 *  designatedflash are will be used in a round-robin manner to maximize flash
 *  endurance.
 *  @param BLOCK_SIZE: Defines how many bytes shall be flashed in one
 *  operation. Usually a small integer, at least 4, defined by the hardware
 *  limitations of the MCU flash (for example on the NXP 17xx it is 16 bytes,
 *  because programming less than 16 bytes in one go is not supported).
 *  @param BYTES_PER_BLOCK: how many bytes of actual data should be stored in a
 *  block. Must be <= BLOCK_SIZE - 2 (in order to leave space for the address
 *  in the block). Must be a power of two.
 *  @param SHADOW_IN_RAM: a boolean, if set to true, a shadow memory are will
 *  be allocated in RAM that will be pre-filled with the entire eeprom
 *  data. Dramatically speeds up reads, because reads will not have to go
 *  through the log anymore.
 *  @param file_size: The total number of bytes held by the emulated eeprom
 *  file. Reads from address 0 .. file_size - 1 will be valid. Must be smaller
 *  than half of one sector, but should be realistically about 35% of the
 *  sector size to avoid too frequent sector erasing.
 *
 * Limitations:
 *
 * At any point in time there is only one active sector. This means that all
 * useful data has to fit in one sector. This limits the max number of bytes to
 * be sector_size / 2. To work around this is makes sense to set the eepromemu
 * sector size to be a multiple of what the microcontroller can erase in one
 * go, and writing the flash_erase in a way that just erases all flash sectors
 * that fall into the chosen eepromemu sector.
 *
 * Since there is only one active sector, it makes fairly little sense to have
 * more than two sectors in total. One spare sector is needed for copying data
 * over in a power-failure-safe manner. More than two sectors will not extend
 * the size of available eepromemu space, just round-robin with using the flash
 * sectors, thereby extending flash lifetime -- which already should not be a
 * problem on modern MCUs.
 *
 * The efficiency is not great: only 25% of the allocated flash space can be
 * used for data storage. Users should leave some additional buffer to avoid
 * too frequent overflowing of sectors.
 *
 * The file size is limited to 64k - BLOCK_SIZE because the address is stored
 * on 2 bytes in each block.
 */
class EEPROMEmulation : public EEPROM
{
protected:
    /** Constructor.
     * @param name device name
     * @param file_size maximum file size that we can grow to.
     */
    EEPROMEmulation(const char *name, size_t file_size);

    /** Destructor.
     */
    ~EEPROMEmulation()
    {
    }

    /** Mount the EEPROM file.  Should be called during construction of the
     * derived class.
     */
    void mount();

    /** Sector size in bytes */
    static const size_t SECTOR_SIZE;

    /** block size in bytes */
    static const size_t BLOCK_SIZE;

private:
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

    /** Total FLASH memory size to use for EEPROM Emulation.  Must be at least
     * 2 sectors large and at least 4x the total amount of EEPROM address space
     * that will be emulated.  Larger sizes will result in greater endurance.
     * must be a macro in order to calculate from link time constants.
     */
    #define FLASH_SIZE ((uintptr_t)(&__eeprom_end - &__eeprom_start))

    /** useful data bytes size in bytes 
     *  @todo maybe this should be a macro of BLOCK_SIZE / 2
     */
    static const size_t BYTES_PER_BLOCK;

    /** number of reserved header blocks */
    static const size_t HEADER_BLOCK_COUNT;

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

    /** metadata indexes for magic block markers */
    enum MagicBlockIndex
    {
         MAGIC_FIRST_INDEX = 0, /**< first metadata block index */
         MAGIC_DIRTY_INDEX = 0, /**< dirty metadata block index */
         MAGIC_INTACT_INDEX, /**< intact metadata block index */
         MAGIC_USED_INDEX, /**< used metadata block index */
         MAGIC_COUNT /**< total metadata block count */
    };

    /** Write to the EEPROM on a native block boundary.
     * @param index block within EEPROM address space to write
     * @param data data to write, array size must be @ref BYTES_PER_BLOCK large
     */
    void write_block(unsigned int index, const uint8_t data[]);

    /** Read from the EEPROM on a native block boundary.
     * @param index bock within EEPROM address space to read
     * @param data location to place read data, array size must be @ref
     *           BYTES_PER_BLOCK large
     * @param return true if any of the data is not "erased", else return false
     */
    bool read_block(unsigned int index, uint8_t data[]);

    /** Get the next active block pointer.
     * @return a pointer to the beginning of the next active block
     */
    uint32_t *next_active()
    {
        return ((activeIndex + 1) < sector_count()) ? sector(activeIndex + 1) :
                                                      sector(0);
    }

    /** Get the active block pointer.
     * @return a pointer to the beginning of the active block
     */
    uint32_t *active()
    {
        return sector(activeIndex);
    }

    /** Get sector data pointer.
     * @param i index of sector to get a data pointer to relative to 1st EEPROM
     *          sector
     * @return a pointer to the beginning of the sector
     */
    uint32_t *sector(int i)
    {
        return sector_to_address(i + address_to_sector(&__eeprom_start));
    }

    /** Get block data pointer based on sector address and block index.
     * @param i = block index within sector
     * @param sector_address absolute sector address to get block from
     * @return a pointer to the beginning of the block
     */
    uint32_t *block(int i, void *sector_address)
    {
        return (uint32_t*)(((uintptr_t)sector_address) + (i * BLOCK_SIZE));
    }

    /** Get the sector index.
     * @param sector_address pointer to the beginning of the sector
     * @return index of sector relative to start of EERPROM region
     */
    int sector_index(uint32_t *sector_address)
    {
        return ((uintptr_t)sector_address - (uintptr_t)&__eeprom_start) / SECTOR_SIZE;
    }

    /** Total number of FLASH sectors being used for emulation.
     * @return number of FLASH sectors being used for emulation
     */
    int sector_count()
    {
        return FLASH_SIZE / SECTOR_SIZE;
    }

    /** Total number of EEPROM slots in a FLASH sector.  A slot is the same
     * as a block, except that it excludes any metadata blocks.
     * @return number of EEPROM slots in a FLASH block.
     */
    int slot_count()
    {
        return (SECTOR_SIZE / BLOCK_SIZE) - MAGIC_COUNT;
    }

    /** Slot data pointer pointing to the last slot in a given sector
     * as a block, except that it excludes any metadata blocks.
     * @param sector_address pointer to the beginning of the sector
     * @return address pointing to the last slot in the sector.
     */
    uint32_t *slot_last(uint32_t *sector_address)
    {
        return sector_address + ((MAGIC_COUNT + slot_count() - 1) *
                                 (BLOCK_SIZE / sizeof(uint32_t)));
    }

    /** Slot data pointer pointing to the first slot in a given sector
     * as a block, except that is exludes any metadata blocks.
     * @param sector_address pointer to the beginning of the sector
     * @return address pointer to the last slot in the sector.
     */
    uint32_t *slot_first(uint32_t *sector_address)
    {
        return sector_address + (MAGIC_COUNT * (BLOCK_SIZE / sizeof(uint32_t)));
    }

    /** Block data pointer pointing to the last magic block in a given sector.
     * @param sector_address pointer to the beginning of the sector
     * @return address pointer to the last magic block in the sector.
     */
    uint32_t *magic_last(uint32_t *sector_address)
    {
        return sector_address + ((MAGIC_COUNT - 1) *
                                 (BLOCK_SIZE / sizeof(uint32_t)));
    }

    /** Lookup sector number from address.
     * @param address sector address;
     * @return sector number.
     */
    virtual int address_to_sector(const void *address) = 0;

    /** Lookup address number from sector number.
     * @param sector sector number;
     * @return sector address.
     */
    virtual uint32_t *sector_to_address(const int sector) = 0;

    /** Simple hardware abstraction for FLASH erase API.
     * @param address the start address of the flash block to be erased
     */
    virtual void flash_erase(void *address) = 0;

    /** Simple hardware abstraction for FLASH program API.
     * @param data a pointer to the data to be programmed
     * @param address the starting address in flash to be programmed.
     *                Must be a multiple of BLOCK_SIZE
     * @param count the number of bytes to be programmed.
     *              Must be a multiple of BLOCK_SIZE
     */
    virtual void flash_program(uint32_t *data, void *address, uint32_t count) = 0;

    /** local copy of SHADOW_IN_RAM which we can manipulate at run time */
    bool shadow_in_ram;

    /** pointer to RAM for shadowing EEPROM. */
    uint8_t *shadow;

    /** index of the active block */
    int activeIndex;

    /** number of available slots for new data */
    size_t available;

    /** Default constructor.
     */
    EEPROMEmulation();

    DISALLOW_COPY_AND_ASSIGN(EEPROMEmulation);
};

#endif /* _FREERTOS_DRIVERS_COMMON_EEPROMEMULATION_HXX_ */
