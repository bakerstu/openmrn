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
 * \file ConfigEntry.hxx
 *
 * Configuration option reader classes.
 *
 * @author Balazs Racz
 * @date 31 May 2015
 */

#ifndef _NMRANET_CONFIGENTRY_HXX_
#define _NMRANET_CONFIGENTRY_HXX_

#include <sys/types.h>
#include <stdint.h>
#include <endian.h>

#include "nmranet/ConfigRenderer.hxx"

namespace nmranet
{

/// Class representing a particular location in the configuration space. All
/// typed configuration objects (atoms as well as groups) will be subclasses of
/// this.
class ConfigReference
{
public:
    /// Initializes the config reference from a configuration space offset.
    ///
    /// @param offset is the integer offset (0-based) in the address space for
    /// configuration.
    constexpr explicit ConfigReference(unsigned offset)
        : offset_(offset)
    {
    }

    /// Initializes the config reference from an existing config reference.
    ///
    /// @param ref is the existing (or saved) config reference.
    constexpr explicit ConfigReference(const ConfigReference& ref)
        : offset_(ref.offset())
    {
    }

    constexpr unsigned offset() {
        return offset_;
    }

protected:
    /// zero-based offset from the beginning of the configuration file.
    unsigned offset_;
};

///
/// Base class for individual configuration entries. Defines helper methods for
/// reading and writing.
///
class ConfigEntryBase : public ConfigReference
{
public:
    using ConfigReference::ConfigReference;

protected:
    /// Reads a given typed variable from the configuration file. DOes not do
    /// any binary conversion (only reads raw data).
    ///
    /// @param fd file to read data from.
    ///
    /// @return the raw value read from the configuration file.
    ///
    template <class T> T raw_read(int fd) const
    {
        T ret;
        repeated_read(fd, &ret, sizeof(T));
        return ret;
    }

private:
    /// Performs a reliable read from the given FD. Crashes if the read fails.
    ///
    /// @param fd the file to read data from
    /// @param buf the location to write data to
    /// @param size how many bytes to read
    ///
    void repeated_read(int fd, void *buf, size_t size) const;
};

template <class TR> class NumericConfigEntry : public ConfigEntryBase
{
public:
    using ConfigEntryBase::ConfigEntryBase;

    /// Performs endian conversion.
    ///
    /// @param d network-byte-order data
    ///
    /// @return host-byte-order data
    ///
    static uint8_t endian_convert(uint8_t d)
    {
        return d;
    }
    /// Performs endian conversion.
    ///
    /// @param d network-byte-order data
    ///
    /// @return host-byte-order data
    ///
    static uint16_t endian_convert(uint16_t d)
    {
        return be16toh(d);
    }
    /// Performs endian conversion.
    ///
    /// @param d network-byte-order data
    ///
    /// @return host-byte-order data
    ///
    static uint32_t endian_convert(uint32_t d)
    {
        return be32toh(d);
    }
    /// Performs endian conversion.
    ///
    /// @param d network-byte-order data
    ///
    /// @return host-byte-order data
    ///
    static uint64_t endian_convert(uint64_t d)
    {
        return be64toh(d);
    }

    /// Storage bytes occupied by the instance in the config file.
    ///
    /// @return number of bytes that the config parser offset will be
    /// incremented by this entry.
    ///
    static constexpr unsigned size()
    {
        return sizeof(TR);
    }

    constexpr AtomConfigRenderer config_renderer() {
        return AtomConfigRenderer("int", size());
    }

    /// Reads the data from the configuration file.
    ///
    /// @param fd file descriptor of the config file.
    ///
    /// @return value of the configuration atom that *this represents.
    ///
    TR read(int fd) const
    {
        return endian_convert(raw_read<TR>(fd));
    }
};

using Uint8ConfigEntry = NumericConfigEntry<uint8_t>;
using Uint16ConfigEntry = NumericConfigEntry<uint16_t>;
using Uint32ConfigEntry = NumericConfigEntry<uint32_t>;
using Uint64ConfigEntry = NumericConfigEntry<uint64_t>;

class EventConfigEntry : public Uint64ConfigEntry {
public:
    using Uint64ConfigEntry::Uint64ConfigEntry;

    constexpr AtomConfigRenderer config_renderer() {
        return AtomConfigRenderer("eventid", AtomConfigRenderer::SKIP_SIZE);
    }
};

} // namespace nmranet

#endif // _NMRANET_CONFIGENTRY_HXX_
