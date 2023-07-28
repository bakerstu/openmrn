/** \copyright
 * Copyright (c) 2023, Balazs Racz
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
 * \file TiFlash.hxx
 *
 * Implementation for TI internal flash for Tiva and CC32xxSF devices. This
 * class is intended to be used by other device drivers.
 *
 * @author Balazs Racz
 * @date 23 Apr 2023
 */

#ifndef _FREERTOS_DRIVERS_TI_TIFLASH_HXX_
#define _FREERTOS_DRIVERS_TI_TIFLASH_HXX_

#ifdef TI_DUAL_BANK_FLASH

/// Flash configuration register.
#define FLASH_CONF 0x400FDFC8
/// This bit in the FLASH_CONF register means that the banks are reversed by
/// their address mapping.
#define FCMME 0x40000000
/// This value defines up to one bit that needs to be XOR-ed to the flash
/// address before calling the flash APIs to cover for reversed flash banks.
static const unsigned addr_mirror = (HWREG(FLASH_CONF) & FCMME) ? 0x80000 : 0;

#else

static constexpr unsigned addr_mirror = 0;

#endif // Dual bank flash

// Different options for what to set for flash write locking. These are only
// needed to debug when the operating system is misbehaving during flash
// writes.

#if defined(TISPIFFS_LOCK_ALL_INTERRUPTS)

// Global disable interrupts.

#define DI() asm("cpsid i\n")
#define EI() asm("cpsie i\n")

#elif defined(TISPIFFS_LOCK_CRITICAL)

// Critical section (interrupts better than MIN_SYSCALL_PRIORITY are still
// running).

#define DI() portENTER_CRITICAL()
#define EI() portEXIT_CRITICAL()

#elif defined(TISPIFFS_LOCK_BASEPRI_FF)

// Disable interrupts with a priority limit of 0xFF (these are the lowest
// priority interrupts, including the FreeRTOS kernel task switch interrupt).
unsigned ppri;
constexpr unsigned minpri = 0xFF;
#define DI()                                                                   \
    do                                                                         \
    {                                                                          \
        unsigned r;                                                            \
        __asm volatile(" mrs %0, basepri\n mov %1, %2\n msr basepri, %1\n"     \
                       : "=r"(ppri), "=r"(r)                                   \
                       : "i"(minpri)                                           \
                       : "memory");                                            \
    } while (0)
#define EI() __asm volatile(" msr basepri, %0\n" : : "r"(ppri) : "memory")

#elif defined(TISPIFFS_LOCK_NOTICK)

// Disable the systick timer to prevent preemptive multi-tasking from changing
// to a different task.

static constexpr unsigned SYSTICKCFG = 0xE000E010;
#define DI() HWREG(SYSTICKCFG) &= ~2;
#define EI() HWREG(SYSTICKCFG) |= 2;

#elif defined(TISPIFFS_LOCK_SCHEDULER_SUSPEND)

// Disable freertos scheduler

#define DI() vTaskSuspendAll()
#define EI() xTaskResumeAll()

#elif defined(TISPIFFS_LOCK_NONE)

// No write locking.

#define DI()
#define EI()

#elif defined(TISPIFFS_LOCK_CRASH)

// Crashes if two different executions of this locking mechanism are
// concurrent.

unsigned pend = 0;
#define DI()                                                                   \
    HASSERT(pend == 0);                                                        \
    pend = 1;
#define EI() pend = 0;

#else
#error Must specify what kind of locking to use for TISPIFFS.
#endif

// This ifdef decides whether we use the ROM or the flash based implementations
// for Flash write and erase. It also supports correcting for the reversed bank
// addresses.
#if 1
#define FPG(data, addr, size) ROM_FlashProgram(data, (addr) ^ addr_mirror, size)
#define FER(addr) ROM_FlashErase((addr) ^ addr_mirror)
#else
#define FPG(data, addr, size) FlashProgram(data, (addr) ^ addr_mirror, size)
#define FER(addr) FlashErase((addr) ^ addr_mirror)
#endif

template <uint32_t ERASE_PAGE_SIZE> class TiFlash
{
public:
    constexpr TiFlash()
    { }

    /// Performs write to the device. This call is synchronous; does not return
    /// until the write is complete.
    /// @param addr where to write (in the address space of the MCU).
    /// @param buf data to write
    /// @param len how many bytes to write
    static void write(uint32_t addr, const void *buf, uint32_t len)
    {
        // Theory of operation:
        //
        // The hardware has 32 word long write buffer. This is aligned based on
        // the flash address that's being written. We split the writes on the
        // boundary of this buffer (which is the low 7 bits of the address).
        //
        // For each buffer, we identify which words are touched by the
        // write. We load these words (32 bits at a time), then we AND the data
        // to be written into it (one byte at a time).
        //
        // Then we program from the buffer using the driverlib API.
        //
        // This algorithm is agnostic to the alignment of the actual writes,
        // and we also guarantee that we never attempt to set a 0 bit to a 1
        // bit in the flash. It also makes optimal use of the parallel write
        // capability of the flash hardware.
        static constexpr uint32_t BUF_WORD_LEN = 32;
        static constexpr uint32_t BUF_BYTE_LEN = 128;
        static constexpr uint32_t BUF_BYTE_MASK = BUF_BYTE_LEN - 1;
        static constexpr uint32_t BYTE_PER_WORD = 4;
        typedef union
        {
            uint32_t w[BUF_WORD_LEN];
            uint8_t b[BUF_BYTE_LEN];
        } BufType;
        static BufType wrbuf;
        
        const uint8_t *src = (const uint8_t *)buf;

        while (len)
        {
            // memory address of the first byte in the buffer.
            uint32_t *buf_address = (uint32_t *)(addr & ~BUF_BYTE_MASK);
            // first word in the buffer that we need to program
            uint32_t buf_word_ofs = (addr & BUF_BYTE_MASK) / BYTE_PER_WORD;
            // first byte in the buffer that we need to load
            uint32_t buf_byte_ofs = (addr & BUF_BYTE_MASK);
            // how many bytes to copy to this buffer.
            uint32_t buf_byte_count =
                std::min((uint32_t)len, BUF_BYTE_LEN - buf_byte_ofs);
            // first word that does not need to be touched
            uint32_t buf_word_end =
                (buf_byte_ofs + buf_byte_count + BYTE_PER_WORD - 1) /
                BYTE_PER_WORD;
            // how many words do we need to program.
            uint32_t buf_word_count = buf_word_end - buf_word_ofs;

            // Pre-fill data with existing flash content.
            for (uint32_t i = 0; i < buf_word_count; ++i)
            {
                wrbuf.w[buf_word_ofs + i] = buf_address[buf_word_ofs + i];
            }

            // Copy the to-be-programmed data with AND operator.
            for (uint32_t i = 0; i < buf_byte_count; ++i)
            {
                wrbuf.b[buf_byte_ofs + i] &= src[i];
            }

            // Program the buffer to flash.
            DI();
            HASSERT(FPG(wrbuf.w + buf_word_ofs,
                        ((uint32_t)buf_address) + buf_word_ofs * BYTE_PER_WORD,
                        buf_word_count * BYTE_PER_WORD) == 0);
            EI();

            // Go to the next flash buffer page.
            len -= buf_byte_count;
            addr += buf_byte_count;
            src += buf_byte_count;
        }
    }

    /// Reads data from the device.
    /// @param addr where to read from (address space of the MCU)
    /// @param buf points to where to put the data read
    /// @param len how many bytes to read
    static void read(uint32_t addr, void *buf, size_t len)
    {
        memcpy(buf, (void *)addr, len);
    }

    /// Aligns an address to the next possible sector start (i.e., rounds up to
    /// sector boundary).
    /// @param addr an address in the flash address space.
    /// @return If addr is the first byte of a sector, then returns addr
    /// unmodified. Otherwise returns the starting address of the next sector.
    static uint32_t next_sector_address(uint32_t addr)
    {
        return (addr + ERASE_PAGE_SIZE - 1) & ~(ERASE_PAGE_SIZE - 1);
    }

    /// Erases sector(s) of the device.
    /// @param addr beginning of the sector to erase. Must be sector aligned.
    /// @param len how many bytes to erase (must be multiple of sector size).
    static void erase(uint32_t addr, size_t len)
    {
        while (len)
        {
            DI();
            HASSERT(FER(addr) == 0);
            EI();
            addr += ERASE_PAGE_SIZE;
            len -= ERASE_PAGE_SIZE;
        }
    }
};

#undef DI
#undef EI
#undef FPG
#undef FER

#endif // _FREERTOS_DRIVERS_TI_TIFLASH_HXX_
