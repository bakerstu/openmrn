/** \copyright
 * Copyright (c) 2016, Balazs Racz
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
 * \file CC32xxEEPROMEmulation.hxx
 * This file implements EEPROM emulation on the CC32xx SPI-flash filesystem.
 *
 * @author Balazs Racz
 * @date 30 December 2016
 */

#ifndef _FREERTOS_DRIVERS_TI_CC32XXEEPROMEMULATION_HXX_
#define _FREERTOS_DRIVERS_TI_CC32XXEEPROMEMULATION_HXX_

//#define FLASH_SIZE CC32xx_EEPROM_SIZE
#include "EEPROMEmulation.hxx"
#include "freertos_drivers/ti/CC32xxHelper.hxx"

/** Emulates EEPROM in SPI-FLASH for the CC32xx platform.
 *
 * Theory of operation:
 *
 * We use one device file per sector. A sector, when erased, will start the
 * file fresh and empty, in a read-write mode. The file will be in read-write
 * mode until the device reboots. Upon the next boot, the files are opened in a
 * read-only mode, and the available() value is set to zero. This will force
 * upon the first write a new sector to be erased and all data to be copied
 * over to it.
 *
 * When the EEPROM driver wants to read a block, we load the data from the file
 * into an in-memory cache, and return a pointer into that cache. This is how
 * we map the file into memory. Fortunately we receive a call before the driver
 * needs a different block, so we get a chance to update the memory mapped
 * region.
 *
 * One major difference is that there is no way to write the "used" block to
 * the past file when the data has been copied over to the new sector. In spite
 * of this we are using a different scheme for the used magic, and fake the
 * writing of the used magic when data is loaded into the cache. The USED magic
 * is virtually set for every sector except the sector last erased that has
 * INTACT data. When we erase a sector we write a "rotation number" to the USED
 * magic offset. The rotation number is incremented every time we jump from
 * sector N-1 to sector 0. This way we can determine upon boot which sector was
 * last erased.
 *
 */
class CC32xxEEPROMEmulation : public EEPROMEmulation
{
public:
    /** Constructor.
     * @param name device name
     * @param file_size_bytes maximum file size that we can grow to.
     */
    CC32xxEEPROMEmulation(const char *name, size_t file_size_bytes);

    /** Destructor.
     */
    ~CC32xxEEPROMEmulation();

private:
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
     * @return pointer to the beginning of the data in the block. Must be alive
     * until the next call to this function.
     */
    const uint32_t *block(unsigned sector, unsigned offset) override;

    /// @return the file offset where the rotation byte is written.
    size_t get_rotation_offset()
    {
        return MAGIC_USED_INDEX * BLOCK_SIZE;
    }

    /// Opens a SL file.
    ///
    /// @param sector number suffix of the filename
    /// @param open_mode one of the SimpleLink constants FS_MODE_*
    /// @param ignore_error when false, HASSERT fails upon error
    ///
    /// @return file handle. checked that it is >= 0
    ///
    int open_file(
        unsigned sector, uint32_t open_mode, bool ignore_error = false);

    /** Default constructor.
     */
    CC32xxEEPROMEmulation();

public:
    /// How many bytes we cache in one read from the filesystem.
    static constexpr unsigned CACHE_SIZE = 256;

    /// A constexpr value for BLOCK_SIZE.
    static constexpr unsigned INT_BLOCK_SIZE = 16;
    /// How may blocks does the cache hold.
    static constexpr unsigned CACHE_BLOCK_COUNT = CACHE_SIZE / INT_BLOCK_SIZE;
    /// Bitmask to AND on a raw block index to get the cache line it belongs
    /// to.
    static constexpr unsigned CACHE_BLOCK_MASK =
        ~((CACHE_SIZE / INT_BLOCK_SIZE) - 1);

private:
    /// When overwriting files we are adding an always increasing number to one
    /// of the magic slots. This allows us to determine where we are with the
    /// overwrites. We use this to fake the USED magic slot.
    uint8_t fileRotation_;
    /// The sector number of the last intact (i.e. not USED) sector.
    uint8_t intactSector_{0xff};
    
    /// The sector number of the read-only file.
    uint8_t roSector_{0xff};
    /// The sector number for the read-write file.
    uint8_t rwSector_{0xff};
    /// The SL file descriptor for the read-only file.
    int32_t roSLFileHandle_{-1};
    /// The SL file descriptor for the read-only file.
    int32_t rwSLFileHandle_{-1};

    /// Which sector we have populated the cache from.
    uint8_t cacheSector_{0xff};
    /// Starting offset of the cache in the device file. This is the raw block
    /// number of the beginning of the cache.
    uint16_t cacheStartBlock_{0};
    /// The cached data.
    uint32_t cache_[CACHE_SIZE / 4];

    /// Multiplier for a block index to get the beginning of that block in the
    /// cache (using the cache_ variables native indexing).
    static constexpr unsigned CACHE_BLOCK_MULT =
        INT_BLOCK_SIZE / sizeof(cache_[0]);

    DISALLOW_COPY_AND_ASSIGN(CC32xxEEPROMEmulation);
};

#endif // _FREERTOS_DRIVERS_TI_CC32XXEEPROMEMULATION_HXX_
