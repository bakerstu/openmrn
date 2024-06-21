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
 * \file SPIFlash.cxx
 *
 * Shared implementation for operating spiflash devices. This class is intended
 * to be used by other device drivers.
 *
 * @author Balazs Racz
 * @date 4 Dec 2021
 */

//#define LOGLEVEL INFO

#include "freertos_drivers/common/SPIFlash.hxx"

#include <fcntl.h>
#include <spi/spidev.h>
#include <stropts.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "os/OS.hxx"
#include "utils/logging.h"

/// Conditional OSMutexLock which can handle a nullptr as mutex (in which case
/// it does not lock anything).
class LockIfExists
{
public:
    LockIfExists(OSMutex *mu)
        : mu_(mu)
    {
        if (mu_)
        {
            mu_->lock();
        }
    }

    ~LockIfExists()
    {
        if (mu_)
        {
            mu_->unlock();
        }
    }

private:
    OSMutex *mu_;
};

#define LockIfExists(l) int error_omitted_mutex_lock_variable[-1]

void SPIFlash::init(const char *dev_name)
{
    spiFd_ = ::open(dev_name, O_RDWR);
    HASSERT(spiFd_ >= 0);

    uint8_t spi_bpw = 8;
    int ret;
    ret = ::ioctl(spiFd_, SPI_IOC_WR_MODE, &cfg_->spiMode_);
    HASSERT(ret == 0);
    ret = ::ioctl(spiFd_, SPI_IOC_WR_BITS_PER_WORD, &spi_bpw);
    HASSERT(ret == 0);
    ret = ::ioctl(spiFd_, SPI_IOC_WR_MAX_SPEED_HZ, &cfg_->speedHz_);
    HASSERT(ret == 0);
}

void SPIFlash::get_id(char id_out[3])
{
    LockIfExists l(lock_);
    struct spi_ioc_transfer xfer[2] = {0, 0};
    xfer[0].tx_buf = (uintptr_t)&cfg_->idCommand_;
    xfer[0].len = 1;
    xfer[1].rx_buf = (uintptr_t)id_out;
    xfer[1].len = 3;
    xfer[1].cs_change = true;
    ::ioctl(spiFd_, SPI_IOC_MESSAGE(2), xfer);
}

void SPIFlash::read(uint32_t addr, void *buf, size_t len)
{
    LockIfExists l(lock_);
    struct spi_ioc_transfer xfer[2] = {0, 0};
    uint8_t rdreq[5];
    rdreq[0] = cfg_->readCommand_;
    rdreq[1] = (addr >> 16) & 0xff;
    rdreq[2] = (addr >> 8) & 0xff;
    rdreq[3] = (addr)&0xff;
    rdreq[4] = 0;
    xfer[0].tx_buf = (uintptr_t)rdreq;
    xfer[0].len = 4 + cfg_->readNeedsStuffing_;
    xfer[1].rx_buf = (uintptr_t)buf;
    xfer[1].len = len;
    xfer[1].cs_change = true;
    ::ioctl(spiFd_, SPI_IOC_MESSAGE(2), xfer);

    auto db = (const uint8_t *)buf;
    LOG(INFO, "read [%x]=%02x%02x%02x%02x, %u bytes success", (unsigned)addr,
        db[0], db[1], db[2], db[3], len);
}

void SPIFlash::write(uint32_t addr, const void *buf, size_t size_bytes)
{
    LockIfExists l(lock_);
    const size_t page_size = (~cfg_->pageSizeMask_) + 1;
    const uint8_t *d = (const uint8_t *)buf;
    while (size_bytes)
    {
        size_t len = std::min(page_size, size_bytes);
        if ((addr & cfg_->pageSizeMask_) !=
            ((addr + len - 1) & cfg_->pageSizeMask_))
        {
            // Anything that overflows to the next page we don't write now.
            len -= (addr + len) & ~cfg_->pageSizeMask_;
        }

        HASSERT((addr & cfg_->pageSizeMask_) ==
            ((addr + len - 1) & cfg_->pageSizeMask_));

        struct spi_ioc_transfer xfer[3] = {0, 0, 0};
        uint8_t wreq[4];
        wreq[0] = cfg_->writeCommand_;
        wreq[1] = (addr >> 16) & 0xff;
        wreq[2] = (addr >> 8) & 0xff;
        wreq[3] = addr & 0xff;
        xfer[0].tx_buf = (uintptr_t)&cfg_->writeEnableCommand_;
        xfer[0].len = 1;
        xfer[0].cs_change = true;
        xfer[1].tx_buf = (uintptr_t)wreq;
        xfer[1].len = 4;
        xfer[2].tx_buf = (uintptr_t)d;
        xfer[2].len = len;
        xfer[2].cs_change = true;
        ::ioctl(spiFd_, SPI_IOC_MESSAGE(3), xfer);

        unsigned waitcount = wait_for_write();
        LOG(VERBOSE,
            "write [%x]=%02x%02x%02x%02x, %u bytes success after %u iter",
            (unsigned)addr, d[0], d[1], d[2], d[3], len, waitcount);

        size_bytes -= len;
        addr += len;
        d += len;
    }
}

unsigned SPIFlash::wait_for_write()
{
    // Now we wait for the write to be complete.
    unsigned waitcount = 0;
    while (true)
    {
        struct spi_ioc_transfer sxfer = {0};
        uint8_t streq[2];
        streq[0] = cfg_->statusReadCommand_;
        streq[1] = 0xFF;
        sxfer.tx_buf = (uintptr_t)streq;
        sxfer.rx_buf = (uintptr_t)streq;
        sxfer.len = 2;
        sxfer.cs_change = true;
        ::ioctl(spiFd_, SPI_IOC_MESSAGE(1), &sxfer);

        if ((streq[1] & cfg_->statusWritePendingBit_) == 0)
        {
            return waitcount;
        }
        waitcount++;
    }
}

void SPIFlash::erase(uint32_t addr, size_t len)
{
    size_t end = addr + len;
    while (addr < end)
    {
        struct spi_ioc_transfer xfer[2] = {0, 0};
        uint8_t ereq[4];
        ereq[0] = cfg_->eraseCommand_;
        ereq[1] = (addr >> 16) & 0xff;
        ereq[2] = (addr >> 8) & 0xff;
        ereq[3] = (addr)&0xff;
        xfer[0].tx_buf = (uintptr_t)&cfg_->writeEnableCommand_;
        xfer[0].len = 1;
        xfer[0].cs_change = true;
        xfer[1].tx_buf = (uintptr_t)ereq;
        xfer[1].len = 4;
        xfer[1].cs_change = true;

        ::ioctl(spiFd_, SPI_IOC_MESSAGE(2), &xfer);

        unsigned waitcount = wait_for_write();
        LOG(INFO, "erase at %x, success after %u iter", (unsigned)addr,
            waitcount);

        addr += cfg_->sectorSize_;
    }
}

void SPIFlash::chip_erase()
{
    struct spi_ioc_transfer xfer[2] = {0, 0};
    xfer[0].tx_buf = (uintptr_t)&cfg_->writeEnableCommand_;
    xfer[0].len = 1;
    xfer[0].cs_change = true;
    xfer[1].tx_buf = (uintptr_t)&cfg_->chipEraseCommand_;
    xfer[1].len = 1;
    xfer[1].cs_change = true;

    ::ioctl(spiFd_, SPI_IOC_MESSAGE(2), &xfer);

    unsigned waitcount = wait_for_write();
    LOG(INFO, "chip-erase, success after %u iter", waitcount);
}
