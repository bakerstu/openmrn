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
 *  @param EEPROMEMU_FLASH_SIZE: Automatically detected from the linker symbols.
 *  An integer (at least 2) multiple of SECTOR_SIZE. Sectors within the
 *  designatedflash are will be used in a round-robin manner to maximize flash
 *  endurance.
 *  @param BLOCK_SIZE: Defines how many bytes shall be flashed in one
 *  operation. Usually a small integer, at least 4, defined by the hardware
 *  limitations of the MCU flash (for example on the NXP 17xx it is 16 bytes,
 *  because programming less than 16 bytes in one go is not supported).
 *  @param BYTES_PER_BLOCK: how many bytes of actual data should be stored in a
 *  block. Must be <= BLOCK_SIZE - 2 (in order to leave space for the address
 *  in the block). Must be a power of two.
 *  @param SHADOW_IN_RAM: a boolean, if set to true, a shadow_ memory are will
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

    /** Maximum byte size of a single block. */
    static constexpr unsigned MAX_BLOCK_SIZE = 16;

private:
    /** This function will be called after every write. The default
     * implementation is a weak symbol with an empty function. It is intended
     * to be overridden in the application to get callbacks for eeprom writes
     * that can trigger a reload. */
    void updated_notification();

    /** Write to the EEPROM.  NOTE!!! This is not necessarily atomic across
     * byte boundaries in the case of power loss.  The user should take this
     * into account as it relates to data integrity of a whole block.
     * @param offset index within EEPROM address space to start write
     * @param buf data to write
     * @param len length in bytes of data to write
     */
    void write(unsigned int offset, const void *buf, size_t len) OVERRIDE;

    /** Read from the EEPROM.
     * @param offset index within EEPROM address space to start read
     * @param buf location to post read data
     * @param len length in bytes of data to read
     */
    void read(unsigned int offset, void *buf, size_t len) OVERRIDE;

    /** Total FLASH memory size to use for EEPROM Emulation.  Must be at least
     * 2 sectors large and at least 4x the total amount of EEPROM address space
     * that will be emulated.  Larger sizes will result in greater endurance.
     * must be a macro in order to calculate from link time constants.
     */
    #define EEPROMEMU_FLASH_SIZE ((uintptr_t)(&__eeprom_end - &__eeprom_start))
    
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

protected:
    /** magic marker for an intact block */
    static const uint32_t MAGIC_INTACT;

    /** magic marker for a block that we are transitioning to intact */
    static const uint32_t MAGIC_DIRTY;

    /** magic marker for a used block */
    static const uint32_t MAGIC_USED;

    /** magic marker for an erased block */
    static const uint32_t MAGIC_ERASED;

    /** Raw block indexes within a sector to mark the state of the sector. */
    enum MagicBlockIndex
    {
         MAGIC_FIRST_INDEX = 0, /**< first metadata block index */
         /** dirty metadata block index: programmed when we start writing to the sector (i.e. it is not erased). */
         MAGIC_DIRTY_INDEX = 0,
         /** intact metadata block index: programmed when the data in this sector is authoritative. */
         MAGIC_INTACT_INDEX,
         /** used metadata block index: programmed when the data has been copied to a new sector. */
         MAGIC_USED_INDEX,
         MAGIC_COUNT /**< total metadata block count */
    };

private:
    /** Write to the EEPROM on a native block boundary.
     * @param index block within EEPROM address space to write
     * @param data data to write, array size must be @ref BYTES_PER_BLOCK large
     */
    void write_fblock(unsigned int index, const uint8_t data[]);

    /** Read from the EEPROM on a native block boundary.
     * @param index block within EEPROM address space to read
     * @param data location to place read data, array size must be @ref
     *           BYTES_PER_BLOCK large
     * @return true if any of the data is not "erased", else return false
     */
    bool read_fblock(unsigned int index, uint8_t data[]);

    /** Get the next active sector pointer.
     * @return sector index for the next sector to use.
     */
    unsigned next_active()
    {
    	unsigned next = activeSector_ + 1;
    	if (next >= sectorCount_) next = 0;
    	return next;
    }

    /** Total number of FLASH sectors being used for emulation.
     * @return number of FLASH sectors being used for emulation
     */
    unsigned sector_count()
    {
        return sectorCount_;
    }

    /** Total number of EEPROM slots in a FLASH sector.  A slot is a block
     * that contains user data (i.e., excludes any metadata blocks).
     * @return number of EEPROM slots in a FLASH sector.
     */
    unsigned slot_count()
    {
        return rawBlockCount_ - MAGIC_COUNT;
    }

    /**
     * @return raw block index of the last block containing user data.
     */
    unsigned slot_last()
    {
    	return rawBlockCount_ - 1;
    }

    /**
     * @return raw block index of the first block containing user data.
     */
    unsigned slot_first()
    {
        return MAGIC_COUNT;
    }

    /**
     * Computes the pointer to load the data stored in a specific block from.
     * @param sector sector number [0..sectorCount_ - 1]
     * @param offset block index within sector, [0..rawBlockCount_ - 1]
     * @return pointer to the beginning of the data in the block. Must be alive until the next call to this function.
     */
    virtual const uint32_t* block(unsigned sector, unsigned offset) = 0;

    /** Simple hardware abstraction for FLASH erase API.
     * @param sector Number of sector [0.. sectorCount_ - 1] to erase
     */
    virtual void flash_erase(unsigned sector) = 0;

    /** Simple hardware abstraction for FLASH program API.
     * @param sector the sector to write to [0..sectorCount_ - 1]
     * @param start_block the block index to start writing to [0..rawBlockCount_ - 1]
     * @param data a pointer to the data to be programmed
     * @param byte_count the number of bytes to be programmed.
     *              Must be a multiple of BLOCK_SIZE
     *
     * The bytes to program cannot overflow beyond the end of sector, so
     * start_block + byte_count / BLOCK_SIZE <= rawBlockCount_ must hold.
     */
    virtual void flash_program(unsigned sector, unsigned start_block, uint32_t *data, uint32_t byte_count) = 0;

protected:
    /** Total number of sectors available. */
    const uint8_t sectorCount_{(uint8_t)(EEPROMEMU_FLASH_SIZE / SECTOR_SIZE)};

    /** Index of the active sector. */
    uint8_t activeSector_{0};

    /** How many blocks are there in a sector. */
    const uint16_t rawBlockCount_{(uint16_t)(SECTOR_SIZE / BLOCK_SIZE)};

    /** Number of available (writable) slots for new data in the active sector. */
    size_t availableSlots_{0};

    /** local copy of SHADOW_IN_RAM which we can manipulate at run time. Specifies whether the shadowing is active. */
    bool shadowInRam_{false};

    /** pointer to RAM for shadowing EEPROM. */
    uint8_t *shadow_{nullptr};


    /** Default constructor.
     */
    EEPROMEmulation();

    DISALLOW_COPY_AND_ASSIGN(EEPROMEmulation);
};

#endif /* _FREERTOS_DRIVERS_COMMON_EEPROMEMULATION_HXX_ */
