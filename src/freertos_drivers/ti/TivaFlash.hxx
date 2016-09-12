/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file TivaFlash.hxx
 * This file implements a Flash-backed file on the TI Tiva controllers.
 *
 * @author Balazs Racz
 * @date 2 May 2015
 */

#ifndef _FREERTOS_DRIVERS_TI_TIVAFLASH_HXX_
#define _FREERTOS_DRIVERS_TI_TIVAFLASH_HXX_

#include "Devtab.hxx"

/** Flash-backed file.
 *
 * The memory in flash for this file has to be reserved by the linker
 * script. Any read operations will do just memcpy from the reserved pointer to
 * the application buffer. Doing direct reads from flash using the reserved
 * pointer is also valid.
 *
 * Only sequential writes are supported. This means that opening the file for
 * write, and writing from the beginning until the end is okay, but any other
 * write access is not guaranteed to work.
 *
 * Specifically, a flash page will be erased if and only if the first byte of
 * that page is being written. */
class TivaFlash : public Node
{
public:
    /// Constructor.
    ///
    /// @param name name of th device/file (e.g. "/etc/automata_block"
    /// @param ptr flash base
    /// @param length how many bytes of the file to reserve
    /// @param page_size how often we should erase
    TivaFlash(
        const char *name, const void *ptr, size_t length, uint32_t page_size)
        : Node(name)
        , base_(static_cast<const uint8_t *>(ptr))
        , len_(length)
        , pageSize_(page_size)
    {
        HASSERT(!(read_address() & (page_size - 1)));
    }

    /** Read method. @return negative errno on failure. */
    ssize_t read(File *, void *, size_t) OVERRIDE;
    /** Write method. @return negative errno on failure. */
    ssize_t write(File *, const void *, size_t) OVERRIDE;

    void enable() OVERRIDE {}
    void disable() OVERRIDE {}
    void flush_buffers() OVERRIDE {}

private:
    /** Returns a bitmask with one bits to get the within-page address */
    uint32_t subpage_mask() {
        return pageSize_ - 1;
    }

    /** Returns a bitmask with one bits to get the page address */
    uint32_t page_mask() {
        return ~subpage_mask();
    }

    /** Returns a bitmask with one bits to get the page address */
    uint32_t write_page_mask() {
        return ~(write_page_size()-1);
    }

    /** Returns a bitmask with one bits to get the page address */
    uint32_t write_page_size() {
        return 0x80;
    }

    /// @return pointer that can be used for reading data from this file
    /// directly.
    const uint8_t* read_pointer() {
        return base_;
    }

    /// @return address (as uint32) of the base pointer.
    uint32_t read_address() {
        return reinterpret_cast<uint32_t>(base_);
    }

    /// Base pointer.
    const uint8_t *base_;
    /// Number of bytes in the file.
    size_t len_;
    /// Page size (how often to erase).
    size_t pageSize_;
};

#endif // _FREERTOS_DRIVERS_TI_TIVAFLASH_HXX_
