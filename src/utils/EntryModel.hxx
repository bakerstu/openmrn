/** @copyright
 * Copyright (c) 2017, Stuart W. Baker
 * All rights reserved
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
 * @file EntryModel.hxx
 * This file represents a decimal/hex number entry field in text.
 *
 * @author Stuart W. Baker
 * @date 1 December 2017
 */

#ifndef _UTILS_ENTRYMODEL_HXX_
#define _UTILS_ENTRYMODEL_HXX_

#include <algorithm>
#include <cstring>
#include <functional>
#include <type_traits>

#include "utils/format_utils.hxx"

/** Implementation of a text entry menu.
 * @tparam T the data type up to 64-bits in size
 */
template <class T>
class EntryModel
{
public:
    /** Constructor.
     * @param upper force characters to be upper case
     * @param clamp_callback callback method to clamp min/max
     */
    EntryModel(bool upper = false,
               std::function<void(bool)> clamp_callback = nullptr)
        : clampCallback_(clamp_callback)
        , value_(0)
        , numLeadingZeros_(0)
        , maxSize_(0)
        , size_(0)
        , isAtInitialValue_(false)
        , empty_(true)
        , upper_(upper)
        , base_(10)
    {
    }

    /** Clear the entry string.
     */
    void clear()
    {
        value_ = 0;
        numLeadingZeros_ = 0;
        size_ = 0;
        empty_ = true;
        isAtInitialValue_ = false;
    }

    /** Initialize empty.
     * @param max_size max number of digits in the base type
     * @param base base type, 10 or 16
     */
    void init(unsigned max_size, int base)
    {
        maxSize_ = max_size;
        clear();
        set_base(base);
    }

    /** Initialize with a value.
     * @param max_size max number of digits in the base type
     * @param base base type, 10 or 16
     * @param value value to initialize with
     */
    void init(unsigned max_size, int base, T value)
    {
        init(max_size, base);
        value_ = value;
        isAtInitialValue_ = true;
        empty_ = false;
    }

    /** Append a value to the "back".
     * @param val value to append, base 10: 0 - 9, base 16: 0x0 - 0xF
     */
    void push_back(uint8_t val)
    {
        HASSERT(val < base_);
        if (size_ >= maxSize_)
        {
            // clear entry, return without appending the character
            clear();
            return;
        }
        if (isAtInitialValue_)
        {
            // clear entry before inserting character
            clear();
        }
        value_ *= base_;
        if (value_ < 0)
        {
            value_ -= val;
        }
        else
        {
            value_ += val;
        }
        if (value_ == 0 && !empty_)
        {
            ++numLeadingZeros_;
        }
        empty_ = false;
        ++size_;
        clamp();
    }

    /** Append a character to the "back".
     * @param c character to append, base 10: 0 - 9, base 16: 0 - F
     */
    void push_back_char(char c)
    {
        switch (base_)
        {
            case 10:
                HASSERT(c >= '0' && c <= '9');
                push_back(c - '0');
                break;
            case 16:
                c = toupper(c);
                HASSERT((c >= '0' && c <= '9') ||
                        (c >= 'A' && c <= 'F'));
                push_back(c <= '9' ?  c - '0' : c - 'A' + 10);
                break;
        }
    }

    /** Append a value to the "back".
     * @param val value to append, base 10: 0 - 9, base 16: 0x0 - 0xF
     * @return *this
     */
    EntryModel &append(uint8_t val)
    {
        push_back(val);
        return *this;
    }

    /** Append a character to the "back".
     * @param c character to append, base 10: 0 - 9, base 16: 0 - F
     * @return *this
     */
    EntryModel &append_char(char c)
    {
        push_back_char(c);
        return *this;
    }

    /** Removes (deletes) a character off the end.
     */
    void pop_back()
    {
        if (value_ == 0 && numLeadingZeros_)
        {
            --numLeadingZeros_;
        }
        else
        {
            value_ /= base_;
        }
        if (size_)
        {
            --size_;
        }
        if (isAtInitialValue_)
        {
            isAtInitialValue_ = false;
            clamp();
            // need to compute the size now that the initial value is false
            if (value_ <= 0)
            {
                size_ = 1;
            }
            for (T tmp = value_; tmp != 0; tmp /= base_)
            {
                ++size_;
            }
        }
        if (size_ == 0)
        {
            // no more characters left, so the entry is "empty"
            empty_ = true;
        }
    }

    /** Set the radix base.
     * @param base new radix base to set.
     */
    void set_base(int base)
    {
        HASSERT(base == 10 || base == 16);
        base_ = base;
    }

    /** Set the value, keep the max number of digits and base the same.
     * @param value value to initialize with
     */
    void set_value(T value)
    {
        init(maxSize_, base_, value);
    }

    /** Get the size (actual number of digits). Note, if the entry is still
     * at its initial value, the result will be 0.
     * @return size actual size in number of digits
     */
    size_t size()
    {
        return size_;
    }

    /** Test if the entry is "empty". Having an initial value is not empty.
     * @return true if empty, else false
     */
    bool empty()
    {
        return (!isAtInitialValue_ && empty_);
    }

    /** Test if cursor is visible.
     * @return true if cursor is visiable, else false
     */
    bool cursor_visible()
    {
        return size_ < maxSize_;
    }

    /** Determine if this object is holding an initial or modified value.
     * @return true if if holding an initial value, else false if modified
     */
    bool is_at_initial_value()
    {
        return isAtInitialValue_;
    }

    /** Get the entry as an unsigned integer value. Note, that '0' is returned
     * both when the actual value is '0' and when the entry is "empty". If the
     * caller needs to distinguish between these two states, check for
     * "empty()".
     * @param force_clamp Normally, clamping doesn't occur if the entry is
     *                    "empty". However, if force is set to true, we will
     *                    clamp anyways. This may be valuable when wanting an
     *                    "empty" entry to return a valid value and '0' is out
     *                    of bounds.
     * @return value representation of the entry
     */
    T get_value(bool force_clamp = false)
    {
        if (force_clamp)
        {
            clamp(true);
        }
        return value_;
    }

    /** Get the value as a string. The number of characters will not be trimmed
     * to maxSize_. If trimming is required, it must be done by the caller.
     * @param right_justify true to right justify.
     */
    string get_string(bool right_justify = false)
    {
        string str;
        if (isAtInitialValue_ || !empty_)
        {
            switch (base_)
            {
                default:
                    // should never get here.
                    break;
                case 10:
                    if (std::is_signed<T>::value)
                    {
                        str = int64_to_string(value_);
                    }
                    else
                    {
                        str = uint64_to_string(value_);
                    }
                    break;
                case 16:
                    if (std::is_signed<T>::value)
                    {
                        str = int64_to_string_hex(value_);
                    }
                    else
                    {
                        str = uint64_to_string_hex(value_);
                    }
                    if (upper_)
                    {
                        /* requires all characters in upper case */
                        transform(str.begin(), str.end(), str.begin(), toupper);
                    }
                    break;
            }
        }
        size_t zeros = std::min(static_cast<size_t>(numLeadingZeros_),
                                maxSize_ - str.size());
        if (zeros)
        {
            str.insert(0, zeros, '0');
        }
        if (right_justify)
        {
            if (str.size() < maxSize_)
            {
                str.insert(0, maxSize_ - str.size(), ' ');
            }
        }
        return str;
    }
    
    /** Change the sign of the data.
     */
    void change_sign()
    {
        if (value_ < 0)
        {
            --size_;
        }
        value_ = -value_;
        if (value_ < 0)
        {
            ++size_;
        }
    }

    /// Clamp the value at the min or max.
    /// @param force Normally, clamping doesn't occur if the entry is "empty".
    ///              However, if force is set to true, we will clamp anyways.
    void clamp(bool force = false)
    {
        if (clampCallback_)
        {
            clampCallback_(force);
        }
    }

private:
    std::function<void(bool)> clampCallback_; /**< callback to clamp value */
    T value_; /**< present value held */

    unsigned numLeadingZeros_  : 5; /**< number of leading zeros */
    unsigned maxSize_          : 5; /**< maximum number of digits */
    unsigned size_             : 5; /**< actual number of digits */
    unsigned isAtInitialValue_ : 1; /**< true if still has the initial value */
    unsigned empty_            : 1; /**< true if the value_ is "empty" */
    unsigned upper_            : 1; /**< force characters to be upper case */
    unsigned reserved_         : 15; /**< reserved bit space */

    int base_; /**< radix base */

    DISALLOW_COPY_AND_ASSIGN(EntryModel);
};
#if 0
/** Specialization of EntryModel with upper and lower bounds
 * @tparam T the data type up to 64-bits in size
 * @tparam N the size of the entry in max number of visible digits.
 */
template <class T, size_t N> class EntryModelBounded : public EntryModel<T, N>
{
public:
    /** Constructor.
     * @param transform force characters to be upper case
     */
    EntryModelBounded(bool transform = false)
        : EntryModel<T, N>(transform,
              std::bind(&EntryModelBounded::clamp, this, std::placeholders::_1))
    {
    }

    /** Initialize with a value.
     * @param digits max number of significant digits in the base type
     * @param base base type, 10 or 16
     * @param value unsigned value to initialize with
     * @param min minumum value
     * @param max maximum value
     * @param default_val default value
     */
    void init(unsigned digits, int base, T value, T min, T max, T default_val)
    {
        HASSERT(default_val >= min && default_val <= max);
        min_ = min;
        max_ = max;
        default_ = default_val;
        EntryModel<T, N>::init(digits, base, value);
    }

    /// Set the value to the minimum.
    void set_min()
    {
        EntryModel<T, N>::set_value(min_);
    }

    /// Set the value to the maximum.
    void set_max()
    {
        EntryModel<T, N>::set_value(max_);
    }

    /// Set the value to the default.
    void set_default()
    {
        EntryModel<T, N>::set_value(default_);
    }

    /// Pre-increment value.
    T operator ++()
    {
        T value = EntryModel<T, N>::get_value();
        if (value < max_)
        {
            ++value;
            EntryModel<T, N>::set_value(value);
        }
        return value;
    }

    /// Pre-decrement value.
    T operator --()
    {
        T value = EntryModel<T, N>::get_value();
        if (value > min_)
        {
            --value;
            EntryModel<T, N>::set_value(value);
        }
        return value;
    }

private:
    /// Clamp the value at the min or max.
    /// @param force Normally, clamping doesn't occur if the entry is "empty".
    ///              However, if force is set to true, we will clamp anyways.
    void clamp(bool force = false)
    {
        if (force || !EntryModel<T, N>::parsed(true).empty())
        {
            volatile T value = EntryModel<T, N>::get_value();
            if (value < min_)
            {
                set_min();
            }
            else if (value > max_)
            {
                set_max();
            }
            else if (EntryModel<T, N>::parsed(true).empty())
            {
                set_default();
            }
        }
    }

    T min_; ///< minimum value
    T max_; ///< maximum value
    T default_; ///< default value

    DISALLOW_COPY_AND_ASSIGN(EntryModelBounded);
};
#endif
#endif /* _UTILS_ENTRYMODEL_HXX_ */

