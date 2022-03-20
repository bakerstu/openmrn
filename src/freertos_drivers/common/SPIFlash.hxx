/** \copyright
 * Copyright (c) 2021, Balazs Racz
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
 * \file SPIFlash.hxx
 *
 * Shared implementation for operating spiflash devices. This class is intended
 * to be used by other device drivers.
 *
 * @author Balazs Racz
 * @date 4 Dec 2021
 */

#ifndef _FREERTOS_DRIVERS_COMMON_SPIFLASH_HXX_
#define _FREERTOS_DRIVERS_COMMON_SPIFLASH_HXX_

#include <inttypes.h>
#include <spi/spidev.h>
#include <sys/types.h>

#include "utils/logging.h"

class OSMutex;
class SPI;

/// Create a const structure like this to tell the spiflash driver how to talk
/// to your spiflash device.
///
/// Use it like this:
///
/// static const SPIFlashConfig cfg = {
///    .speedHz_ = 2000000,
///    .spiMode_ = 3,
///    .writeCommand_ = 0xff,
///};
struct SPIFlashConfig
{
    /// Use this frequency to talk to SPI.
    uint32_t speedHz_ {1000000};

    /// How many bytes is an erase sector.
    uint32_t sectorSize_ {4 * 1024};

    /// A page program operation might wrap around a page. This will cause
    /// bytes to be written to the wrong place. There is a check that prevents
    /// this.
    ///
    /// This variable is the mask on the address bits that define the
    /// page. Each write operation must start and finish within the same
    /// address & pageSizeMask_.
    uint32_t pageSizeMask_ {~(256u - 1)};

    /// SPI mode to use.
    uint8_t spiMode_ {SPI_MODE_0};

    /// Command to use for get identification bytes.
    uint8_t idCommand_ {0x9F};
    /// Command to use for reads.
    uint8_t readCommand_ {0x03};
    /// Command sent out before each write/erase command.
    uint8_t writeEnableCommand_ {0x06};
    /// Command to use for writes.
    uint8_t writeCommand_ {0x02};
    /// Command to use for sector erases.
    uint8_t eraseCommand_ {0x20};
    /// Command to use for chip erase.
    uint8_t chipEraseCommand_ {0x60};

    /// Command to use for status register read.
    uint8_t statusReadCommand_ {0x05};
    /// Which bit to check in the status register for write complete. (This is
    /// a mask, it should have exactly one bit set.)
    uint8_t statusWritePendingBit_ {0x01};

    /// Set this to 1 if the read command needs a dummy byte after the address.
    uint8_t readNeedsStuffing_ : 1;
};

/// Shared implementation for operating spiflash devices. This class is intended
/// to be used by other device drivers.
class SPIFlash
{
public:
    /// Constructor.
    /// @param cfg static configuration for this SPI flash device.
    /// @param lock this lock will be taken before performing any operation on
    /// the chip. Can be null.
    SPIFlash(const SPIFlashConfig *cfg, OSMutex *lock)
        : cfg_(cfg)
        , lock_(lock)
    {
        /// This ensures that the sector size is a power of two.
        HASSERT((cfg->sectorSize_ & (cfg->sectorSize_ - 1)) == 0);
    }

    /// @return the configuration.
    const SPIFlashConfig &cfg()
    {
        return *cfg_;
    }

    /// Opens the SPI bus. This is typically called in hw_postinit or main.
    /// @param dev_name the name of the SPI device.
    void init(const char *dev_name);

    /// Performs write to the device. Thiscall is synchronous; does not return
    /// until the write is complete.
    /// @param addr where to write (0 = beginning of the device).
    /// @param buf data to write
    /// @param len how many bytes to write
    void write(uint32_t addr, const void *buf, size_t len);

    /// Reads data from the device.
    /// @param addr where to read from
    /// @param buf points to where to put the data read
    /// @param len how many bytes to read
    void read(uint32_t addr, void *buf, size_t len);

    /// Aligns an address to the next possible sector start (i.e., rounds up to
    /// sector boundary).
    /// @param addr an address in the flash address space.
    /// @return If addr is the first byte of a sector, then returns addr
    /// unmodified. Otherwise returns the starting address of the next sector.
    uint32_t next_sector_address(uint32_t addr)
    {
        return (addr + cfg_->sectorSize_ - 1) & ~(cfg_->sectorSize_ - 1);
    }

    /// Erases sector(s) of the device.
    /// @param addr beginning of the sector to erase. Must be sector aligned.
    /// @param len how many bytes to erase (must be multiple of sector size).
    void erase(uint32_t addr, size_t len);

    /// Erases the entire device.
    void chip_erase();

    /// Fetches the identification bytes form the SPIFlash.
    /// @param id_out return parameter, will be filled with the received
    /// identification bytes.
    void get_id(char id_out[3]);

private:
    /// Waits until write is complete.
    /// @return how many iterations the wait took
    unsigned wait_for_write();

    /// Configuration.
    const SPIFlashConfig *cfg_;

    /// Lock that protects accesses to the flash chip.
    OSMutex *lock_;

    /// File descriptor for the opened SPI bus.
    int spiFd_ {-1};
    /// Direct access of the SPI device pointer.
    /// @todo maybe we are not actually using this.
    SPI *spi_;
};

#endif // _FREERTOS_DRIVERS_COMMON_SPIFLASH_HXX_
