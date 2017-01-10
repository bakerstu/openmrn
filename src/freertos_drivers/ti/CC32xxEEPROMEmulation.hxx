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
class CC32xxEEPROMEmulation : public EEPROM
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

    /// Instructs the driver to write all changed data to disk.
    void flush() {
        OSMutexLock l(&lock_);
        flush_buffers();
    }

    /// Must be called exactly once after creation, after the simplelink stack
    /// has been started.
    void mount();
    
private:
    /** Write to the EEPROM.  NOTE!!! This is not necessarily atomic across
     * byte boundaries in the case of power loss.  The user should take this
     * into account as it relates to data integrity of a whole block.
     * @param index index within EEPROM address space to start write
     * @param buf data to write
     * @param len length in bytes of data to write
     */
    void write(unsigned int index, const void *buf, size_t len) override;

    /** Read from the EEPROM.
     * @param index index within EEPROM address space to start read
     * @param buf location to post read data
     * @param len length in bytes of data to read
     */
    void read(unsigned int index, void *buf, size_t len) override;

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

    /// REQUIRED: lock_ is held.
    void flush_buffers() OVERRIDE;
    
private:
    /// The total number of files which we are round-robining.
    static const uint8_t SECTOR_COUNT;

    /// This number is increased by one every time the contents are flushed to
    /// a file. The version gets written to the file so that we can recognize
    /// which is the latest file.
    unsigned fileVersion_{0};

    /// The sector number of the last sector we used for reading.
    uint8_t readSector_{0xff};

    /// non-zero if we have unflushed writes.
    uint8_t isDirty_{0};
    
    /// Holds the file payload in memory.
    uint8_t *data_;
};

#endif // _FREERTOS_DRIVERS_TI_CC32XXEEPROMEMULATION_HXX_
