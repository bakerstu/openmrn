/** \copyright
 * Copyright (c) 2020, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file VirtualMemorySpace.hxx
 *
 * Implementation of a memory space where the values are not stored in a
 * contiguous storage area but are read and written via callbacks.
 *
 * @author Balazs Racz
 * @date 10 Aug 2020
 */

#ifndef _OPENLCB_VIRTUALMEMORYSPACE_HXX
#define _OPENLCB_VIRTUALMEMORYSPACE_HXX

#include "openlcb/ConfigEntry.hxx"
#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/MemoryConfig.hxx"
#include "utils/SortedListMap.hxx"

namespace openlcb
{

/// Implementation of a memory space where the values are not stored in a
/// contiguous storage area but are read and written via callbacks.
class VirtualMemorySpace : public MemorySpace
{
public:
    /// @returns whether the memory space does not accept writes.
    bool read_only() override
    {
        return isReadOnly_;
    }
    /// @returns the lowest address that's valid for this block.
    address_t min_address() override
    {
        return minAddress_;
    }
    /// @returns the largest valid address for this block.  A read of 1 from
    /// this address should succeed in returning the last byte.
    address_t max_address() override
    {
        return maxAddress_;
    }

    /// @return the number of bytes successfully written (before hitting end
    /// of space).
    /// @param destination address to write to
    /// @param data to write
    /// @param len how many bytes to write
    /// @param error if set to non-null, then the operation has failed. If the
    /// operation needs to be continued, then sets error to
    /// MemorySpace::ERROR_AGAIN, and calls the Notifiable
    /// @param again when a re-try makes sense. The caller should call write
    /// once more, with the offset adjusted with the previously returned
    /// bytes.
    size_t write(address_t destination, const uint8_t *data, size_t len,
        errorcode_t *error, Notifiable *again) override
    {
        if (destination > maxAddress_ || destination + len <= minAddress_)
        {
            *error = MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
            return 0;
        }
        *error = 0;
        unsigned repeat;
        const DataElement *element = nullptr;
        ssize_t skip = find_data_element(destination, len, &element, &repeat);
        string payload;
        if (skip > 0)
        {
            // Will cause a new call be delivered with adjusted data and len.
            return skip;
        }
        else if (skip < 0)
        {
            // We have some missing bytes that we need to read out first, then
            // can perform the write.
            DIE("unimplemented");
            // if (!element->readImpl_(repeat, &payload,
        }
        HASSERT(element);
        payload.assign(
            (const char *)data, std::min((size_t)len, (size_t)element->size_));
        size_t written_len = payload.size();
        bn_.reset(again);
        element->writeImpl_(repeat, std::move(payload), bn_.new_child());
        if (bn_.abort_if_almost_done())
        {
            return written_len;
        }
        else
        {
            // did not succeed synchronously.
            bn_.notify(); // our slice
            *error = MemorySpace::ERROR_AGAIN;
            return 0;
        }
    }

    /** @returns the number of bytes successfully read (before hitting end of
     * space). If *error is set to non-null, then the operation has failed. If
     * the operation needs to be continued, then sets error to ERROR_AGAIN, and
     * calls the Notifiable @param again when a re-try makes sense. The caller
     * should call read once more, with the offset adjusted with the previously
     * returned bytes. */
    size_t read(address_t source, uint8_t *dst, size_t len, errorcode_t *error,
        Notifiable *again) override
    {
        if (source > maxAddress_ || source + len <= minAddress_)
        {
            *error = MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
            return 0;
        }
        *error = 0;
        unsigned repeat;
        const DataElement *element = nullptr;
        ssize_t skip = find_data_element(source, len, &element, &repeat);
        if (skip > 0)
        {
            memset(dst, 0, skip);
            return skip;
        }
        else if (skip < 0)
        {
            DIE("unimplemented");
        }
        HASSERT(element);
        string payload;
        bn_.reset(again);
        element->readImpl_(repeat, &payload, bn_.new_child());
        if (!bn_.abort_if_almost_done())
        {
            // did not succeed synchronously.
            bn_.notify(); // our slice
            *error = MemorySpace::ERROR_AGAIN;
            return 0;
        }
        payload.resize(element->size_); // pads with zeroes
        size_t data_len = std::min(payload.size(), len);
        memcpy(dst, payload.data(), data_len);
        return data_len;
    }

protected:
    /// Function that will be called for writes.
    /// @param repeat 0 to number of repeats if this is in a repeated
    /// group. Always 0 if not repeated group.
    /// @param contents data payload that needs to be written. The data
    /// bytes of this container start at the address_.
    /// @param done must be notified when the write is complete (possibly
    /// inline).
    using WriteFunction = std::function<void(
        unsigned repeat, string contents, BarrierNotifiable *done)>;
    /// Function that will be called for reads.
    /// @param repeat 0 to number of repeats if this is in a repeated
    /// group. Always 0 if not repeated group.
    /// @param contents the payload to be returned from this variable shall
    /// be written here. Will be zero-padded to size_ bytes if shorter.
    /// @param done must be notified when the read values are ready. The
    /// call will be re-tried in this case.
    /// @return true if the read was successful, false if the read needs to be
    /// re-tried later,
    using ReadFunction = std::function<bool(
        unsigned repeat, string *contents, BarrierNotifiable *done)>;

    /// Setup the address bounds from a single CDI group declaration.
    /// @param group is an instance of a group, for example a segment.
    template <class G> void set_bounds_from_group(const G &group)
    {
        minAddress_ = group.offset();
        maxAddress_ = group.offset() + group.size() - 1;
    }

    /// Expand the address bounds from a single CDI group declaration.
    /// @param group is an instance of a group, for example a segment.
    template <class G> void expand_bounds_from_group(const G &group)
    {
        minAddress_ = std::min(minAddress_, (address_t)group.offset());
        maxAddress_ = std::max(
            maxAddress_, (address_t)(group.offset() + group.size() - 1));
    }

    /// Register an untyped element.
    /// @param address the address in the memory space
    /// @param size how many bytes this elements occupes
    /// @param read_f will be called to read this data
    /// @param write_f will be called to write this data
    void register_element(address_t address, address_t size,
        ReadFunction read_f, WriteFunction write_f)
    {
        elements_.insert(DataElement(address, size, read_f, write_f));
    }

    /// Registers a string typed element.
    /// @param entry is the CDI ConfigRepresentation.
    /// @param read_f will be called to read this data
    /// @param write_f will be called to write this data
    template <unsigned SIZE>
    void register_string(const StringConfigEntry<SIZE> &entry,
        ReadFunction read_f, WriteFunction write_f)
    {
        expand_bounds_from_group(entry);
        register_element(entry.offset(), SIZE, read_f, write_f);
    }

    /// Registers a repeated group. Calling this function means that the
    /// virtual memory space of the group will be looped onto the first
    /// repetition. The correct usage is to register the elements of the first
    /// repetition, then register the repetition itself using this call. Nested
    /// repetitions are not supported (either the outer or the inner repetition
    /// needs to be unrolled and registered for each repeat there).
    /// @param group is the repeated group instance. Will take the start
    /// offset, repeat count and repeat size from it.
    template <class Group, unsigned N>
    void register_repeat(const RepeatedGroup<Group, N> &group)
    {
        RepeatElement re;
        re.start_ = group.offset();
        re.end_ = group.end_offset();
        re.repeatSize_ = Group::size();
        HASSERT(re.repeatSize_ * N == re.end_ - re.start_);
        repeats_.insert(std::move(re));
    }

    /// Bounds for valid addresses.
    address_t minAddress_ = 0xFFFFFFFFu;
    /// Bounds for valid addresses.  A read of length 1 from this address
    /// should succeed in returning the last byte.
    address_t maxAddress_ = 0;
    /// Whether the space should report as RO.
    bool isReadOnly_ = false;

private:
    /// We keep one of these for each variable that was declared.
    struct DataElement
    {
        DataElement(address_t address, address_t size, ReadFunction read_f,
            WriteFunction write_f)
            : address_(address)
            , size_(size)
            , writeImpl_(write_f)
            , readImpl_(read_f)
        {
        }
        /// Base offset of this variable (first repeat only).
        address_t address_;
        /// Size of this variable. This is how many bytes of address space this
        /// variable occupies.
        address_t size_;
        /// Function that will be called for writes.
        WriteFunction writeImpl_;
        /// Function that will be called for reads.
        ReadFunction readImpl_;
    };

    /// STL-compatible comparator function for sorting DataElements.
    struct DataComparator
    {
        /// Sorting operator by address.
        bool operator()(const DataElement &a, const DataElement &b) const
        {
            return a.address_ < b.address_;
        }
        /// Sorting operator by address.
        bool operator()(unsigned a, const DataElement &b) const
        {
            return a < b.address_;
        }
        /// Sorting operator by address.
        bool operator()(const DataElement &a, unsigned b) const
        {
            return a.address_ < b;
        }
    };

    /// Represents a repeated group.
    struct RepeatElement
    {
        /// Offset of the repeated group (first repeat).
        uint32_t start_;
        /// Address bytes per repeat.
        uint32_t repeatSize_;
        /// Address byte after the last repeat.
        uint32_t end_;
    };

    /// STL-compatible comparator function for sorting RepeatElements. Sorts
    /// repeats by the end_ as the key.
    struct RepeatComparator
    {
        /// Sorting operator by end address.
        bool operator()(const RepeatElement &a, const RepeatElement &b) const
        {
            return a.end_ < b.end_;
        }
        /// Sorting operator by end address against a lookup key.
        bool operator()(uint32_t a, const RepeatElement &b) const
        {
            return a < b.end_;
        }
    };

    /// Look up the first matching data element given an address in the virtual
    /// memory space.
    /// @param address byte offset to look up.
    /// @param len how many bytes long range to search from address.
    /// @param ptr will be filled with a pointer to the data element when
    /// found, or filled with nullptr if no data element overlaps with the
    /// given range.
    /// @param repeat output argument, filled with zero or the repetition
    /// number.
    /// @return 0 if an exact match is found. -N if a data element is found,
    /// but N first bytes of this element are not covered. A number N in
    /// [1..len-1] if a data element is found, but this many bytes need to be
    /// skipped from address to arrive at the given data element's offset. len
    /// if there was no data element found (in which case also set ptr to
    /// null).
    ssize_t find_data_element(address_t address, address_t len,
        const DataElement **ptr, unsigned *repeat)
    {
        *repeat = 0;
        *ptr = nullptr;
        bool in_repeat = false;
        address_t original_address = address;
        ElementsType::iterator b = elements_.begin();
        ElementsType::iterator e = elements_.end();
        // Align in the known repetitions first.
        auto rit = repeats_.upper_bound(address);
        if (rit == repeats_.end())
        {
            // not a repeat.
        }
        else
        {
            if (rit->start_ <= address && rit->end_ > address)
            {
                // we are in the repeat.
                unsigned cnt = (address - rit->start_) / rit->repeatSize_;
                *repeat = cnt;
                // re-aligns address to the first repetition.
                address -= cnt * rit->repeatSize_;
                in_repeat = true;
                b = elements_.lower_bound(rit->start_);
                e = elements_.lower_bound(rit->start_ + rit->repeatSize_);
            }
        }

        for (int is_repeat = 0; is_repeat <= 1; ++is_repeat)
        {
            auto it = std::upper_bound(b, e, address, DataComparator());
            if (it != elements_.begin())
            {
                auto pit = it - 1;
                // now: pit->address_ <= address
                if (pit->address_ + pit->size_ > address)
                {
                    // found overlap
                    *ptr = &*pit;
                    return (ssize_t)pit->address_ -
                        (ssize_t)address; // may be negative!
                }
                // else: no overlap, look at the next item
            }
            // now: it->address_ > address
            if (address + len > it->address_)
            {
                // found overlap, but some data needs to be discarded.
                *ptr = &*it;
                return it->address_ - address;
            }

            if (in_repeat)
            {
                // We might be too close to the end of a repetition, we will
                // try with the next repeat instead.
                address -= rit->repeatSize_;
                *repeat += 1;
                if (original_address + rit->repeatSize_ >= rit->end_)
                {
                    // We ran out of repeats. Look at the range beyond the
                    // group instead.
                    b = elements_.lower_bound(rit->end_);
                    e = elements_.end();
                    *repeat = 0;
                }
            }
            else
            {
                break;
            }
        }

        // now: no overlap either before or after.
        return len;
    }

    /// Container type for storing the data elements.
    typedef SortedListSet<DataElement, DataComparator> ElementsType;
    /// Stores all the registered variables.
    ElementsType elements_;
    /// Stores all the registered variables.
    SortedListSet<RepeatElement, RepeatComparator> repeats_;
    /// Helper object in the function calls.
    BarrierNotifiable bn_;
}; // class VirtualMemorySpace

} // namespace openlcb

#endif // _OPENLCB_VIRTUALMEMORYSPACE_HXX
