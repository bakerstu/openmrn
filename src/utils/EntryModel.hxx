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
#include <limits>

#include "utils/format_utils.hxx"

/// Implementation of a text entry menu.
/// @tparam T the data type up to 64-bits in size, signed or unsigned
///
template <class T>
class EntryModel
{
public:
    /// Constructor.
    EntryModel()
        : value_(0)
        , valueMin_(std::numeric_limits<T>::lowest())
        , valueMax_(std::numeric_limits<T>::max())
        , numLeadingZeros_(0)
        , maxSize_(0)
        , size_(0)
        , isAtInitialValue_(false)
        , empty_(true)
        , base_(10)
        , autoClamp_(true)
    {
    }

    /// Clear the entry string.
    void clear()
    {
        value_ = 0;
        numLeadingZeros_ = 0;
        size_ = 0;
        empty_ = true;
        isAtInitialValue_ = false;
    }

    /// Initialize empty.
    /// @param max_size max number of digits in the base type
    /// @param base base type, 10 or 16
    void init(unsigned max_size, int base)
    {
        maxSize_ = max_size;
        autoClamp_ = true;
        clear();
        set_base(base); // this will call set_boundaries()
    }

    /// Initialize with a value.
    /// @param max_size max number of digits in the base type
    /// @param base base type, 10 or 16
    /// @param value value to initialize with
    /// @param automatic_clamp Unless otherwise specified by the API, enables
    ///                        automatic clamping of the value each time it is
    ///                        modified when true, else automatic clamping is
    ///                        not applied.
    void init(unsigned max_size, int base, T value, bool automatic_clamp = true)
    {
        init(max_size, base);
        autoClamp_ = automatic_clamp;
        value_ = value;
        isAtInitialValue_ = true;
        empty_ = false;
    }

    /// Append a value to the "back".
    /// @param val value to append, base 10: 0 - 9, base 16: 0x0 - 0xF
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
        if (value_ == 0 && val != 0 && size_)
        {
            // This is a special case where we transition from a user having
            // entered all 0 (as a first digits) and then enters a non-zero
            // (as a next digit).
            ++numLeadingZeros_;
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
            if (numLeadingZeros_ < 31)
            {
                ++numLeadingZeros_;
            }
        }
        empty_ = false;
        ++size_;
        auto_clamp();
    }

    /// Append a character to the "back".
    /// @param c character to append, base 10: 0 - 9, base 16: 0 - F
    void push_back_char(char c)
    {
        switch (base_)
        {
            case 10:
                push_back(c - '0');
                break;
            case 16:
                c = toupper(c);
                push_back(c <= '9' ?  c - '0' : c - 'A' + 10);
                break;
        }
    }

    /// Append a value to the "back".
    /// @param val value to append, base 10: 0 - 9, base 16: 0x0 - 0xF
    /// @return *this
    EntryModel &append(uint8_t val)
    {
        push_back(val);
        return *this;
    }

    /// Append a character to the "back".
    /// @param c character to append, base 10: 0 - 9, base 16: 0 - F
    /// @return *this
    EntryModel &append_char(char c)
    {
        push_back_char(c);
        return *this;
    }

    /// Removes (deletes) a character off the end.
    void pop_back()
    {
        value_ /= base_;
        if (value_ == 0 && numLeadingZeros_)
        {
            --numLeadingZeros_;
        }
        if (size_)
        {
            --size_;
        }
        if (isAtInitialValue_)
        {
            isAtInitialValue_ = false;
            auto_clamp();
            // need to compute the size now that the initial value is false
            calculate_size();
        }
        if (size_ == 0)
        {
            // no more characters left, so the entry is "empty"
            empty_ = true;
        }
        auto_clamp();
    }

    /// Set the radix base.
    /// @param base new radix base to set.
    void set_base(int base)
    {
        HASSERT(base == 10 || base == 16);
        base_ = base;
        set_boundaries();
    }

    /// Set the radix base.
    /// @param base new radix base to set.
    /// @param convert convert the current value, as a string, to the new base.
    void set_base(int base, bool convert)
    {
        if (base != base_)
        {
            if (convert)
            {
                string str = get_string();
                if (std::is_signed<T>::value)
                {
                    value_ = strtoll(str.c_str(), nullptr, base);
                }
                else
                {
                    value_ = strtoull(str.c_str(), nullptr, base);
                }
            }
            set_base(base);
        }
    }

    /// Set the value, keep the max number of digits and base the same.
    /// @param value value to initialize with
    void set_value(T value)
    {
        init(maxSize_, base_, value, autoClamp_);
    }

    /// Get the size (actual number of digits). Note, if the entry is still
    /// at its initial value, the result will be 0.
    /// @return size actual size in number of digits
    size_t size()
    {
        return size_;
    }

    /// Get the max size (in digits).
    /// @return max size in number of digits
    size_t max_size()
    {
        return maxSize_;
    }

    /// Test if the entry is "empty". Having an initial value is not empty.
    /// @return true if empty, else false
    bool empty()
    {
        return (!isAtInitialValue_ && empty_);
    }

    /// Test if cursor is visible.
    /// @return true if cursor is visible, else false
    bool cursor_visible()
    {
        return size_ < maxSize_;
    }

    /// Determine if this object is holding an initial or modified value.
    /// @return true if if holding an initial value, else false if modified
    bool is_at_initial_value()
    {
        return isAtInitialValue_;
    }

    /// It is not always possible with get_string() to return the leading
    /// zeros. Furthermore, get_value() does not tell the caller if there are
    /// leading zeros. Therefore, this API provides a definitive answer.
    bool has_leading_zeros()
    {
        return numLeadingZeros_ > 0;
    }

    /// Sets the number of leading zeros without changing the value.
    void set_leading_zeros(unsigned num)
    {
        numLeadingZeros_ = num;
    }

    /// Get the entry as an unsigned integer value. Note, that '0' is returned
    /// both when the actual value is '0' and when the entry is "empty". If the
    /// caller needs to distinguish between these two states, check for
    /// "empty()".
    /// @param force_clamp Normally, clamping doesn't occur if the entry is
    ///                    "empty". However, if force is set to true, we will
    ///                    clamp anyways. This may be valuable when wanting an
    ///                    "empty" entry to return a valid value and '0' is out
    ///                    of bounds. The auto clamping setting is ignored.
    /// @return value representation of the entry
    T get_value(bool force_clamp = false)
    {
        if (force_clamp)
        {
            clamp(true);
        }
        return value_;
    }

    /// Get the value as a string. The number of characters will not be trimmed
    /// to maxSize_. If trimming is required, it must be done by the caller.
    /// @param right_justify true to right justify.
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
                    // requires all characters in upper case
                    transform(str.begin(), str.end(), str.begin(), toupper);
                    break;
            }
        }

        if (numLeadingZeros_)
        {
            str.insert(value_ < 0 ? 1 : 0, numLeadingZeros_, '0');
        }
        if (right_justify && str.size() < maxSize_)
        {
            str.insert(0, maxSize_ - str.size(), ' ');
        }
        return str;
    }
    
    /// Set the value to the minimum.
    void set_min()
    {
        set_value(valueMin_);
    }

    /// Set the value to the maximum.
    void set_max()
    {
        set_value(valueMax_);
    }

    /// Change the sign of the data.
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
        auto_clamp();
    }

    /// Clamp the value at the min or max. Clamping will not occur if the value
    /// is zero and there is space for more leading zeros.
    /// @param force Normally, clamping doesn't occur if the entry is "empty".
    ///              However, if force is set to true, we will clamp anyways.
    ///              force also applies if the value is zero yet there is space
    ///              for more leading zeros.
    virtual void clamp(bool force = false)
    {
        if (value_ == 0 && size_ < maxSize_ && !force)
        {
            // skip clamping if we have space for more leading zeros
            return;
        }
        if (force || !empty_)
        {
            empty_ = false;
            if (value_ < valueMin_)
            {
                value_ = valueMin_;
                calculate_size();
            }
            else if (value_ > valueMax_)
            {
                value_ = valueMax_;
                calculate_size();
            }
        }
    }

    /// Pre-increment value. While this method does prevent wrap around of
    /// the native type limits, it is incumbent on the caller to limit the
    /// resulting number of digits. Always clamps, the auto clamping setting
    /// is ignroed.
    T operator ++()
    {
        isAtInitialValue_ = false;
        if (value_ < std::numeric_limits<T>::max())
        {
            ++value_;
        }
        clamp();
        return value_;
    }

    /// Pre-decrement value. While this method does prevent wrap around of
    /// the native type limits, it is incumbent on the caller to limit the
    /// resulting number of digits. Always clamps, the auto clamping setting
    /// is ignroed.
    T operator --()
    {
        isAtInitialValue_ = false;
        if (value_ > std::numeric_limits<T>::lowest())
        {
            --value_;
        }
        clamp();
        return value_;
    }

protected:
    /// Calls clamp() only if automatic clamping is enabled (autoClamp_ = true).
    /// @param force Normally, clamping doesn't occur if the entry is "empty".
    ///              However, if force is set to true, we will clamp anyways.
    ///              force also applies if the value is zero yet there is space
    ///              for more leading zeros.
    void auto_clamp(bool force = false)
    {
        if (autoClamp_)
        {
            clamp(force);
        }
    }

    /// Set min and max boundaries supported based on maxSize_ (digit count).
    virtual void set_boundaries()
    {
        valueMax_ = 0;
        for (unsigned i = 0; i < maxSize_; ++i)
        {
            valueMax_ *= base_;
            valueMax_ += base_ - 1;
        }
        valueMin_ = std::is_signed<T>::value ? valueMax_ / -base_ : 0;
    }

    /// Calculate the size in digits
    void calculate_size()
    {
        // calculate new size_
        size_ = value_ < 0 ? 1 : 0;
        for (T tmp = value_ < 0 ? -value_ : value_; tmp != 0; tmp /= base_)
        {
            ++size_;
        }
        numLeadingZeros_ = std::min(static_cast<unsigned>(numLeadingZeros_),
                                    static_cast<unsigned>(maxSize_ - size_));
        size_ += numLeadingZeros_;
    }

    T value_; ///< present value held
    T valueMin_; ///< minimum value representable by maxSize_
    T valueMax_; ///< maximum value representable by maxSize_ 

    unsigned numLeadingZeros_  : 5; ///< number of leading zeros
    unsigned maxSize_          : 5; ///< maximum number of digits
    unsigned size_             : 5; ///< actual number of digits
    unsigned isAtInitialValue_ : 1; ///< true if still has the initial value
    unsigned empty_            : 1; ///< true if the value_ is "empty"
    unsigned base_             : 6; ///< radix base
    unsigned autoClamp_        : 1; ///< true to auto clamp the values

    DISALLOW_COPY_AND_ASSIGN(EntryModel);
};

/// Specialization of EntryModel with upper and lower bounds
/// @tparam T the data type up to 64-bits in size
/// @tparam N the size of the entry in max number of visible digits.
template <class T> class EntryModelBounded : public EntryModel<T>
{
public:
    /// Constructor.
    EntryModelBounded()
        : EntryModel<T>()
    {
    }

    /// Initialize with a value.
    /// @param max_size max number of digits in the base type
    /// @param base base type, 10 or 16
    /// @param value value to initialize with
    /// @param min minumum value
    /// @param max maximum value
    /// @param default_val default value
    /// @param automatic_clamp
    void init(unsigned max_size, int base, T value, T min, T max, T default_val,
              bool automatic_clamp = true)
    {
        // purposely do not boundary check the min, max, and default values
        EntryModel<T>::init(max_size, base, value, automatic_clamp);
        // override type min/max values
        EntryModel<T>::valueMin_ = min;
        EntryModel<T>::valueMax_ = max;
        valueDefault_ = default_val;
    }

    /// Set the value to the default.
    void set_default()
    {
        EntryModel<T>::set_value(valueDefault_);
    }

private:
    /// Clamp the value at the min or max. Clamping will not occur if the value
    /// is zero and there is space for more leading zeros.
    /// @param force Normally, clamping doesn't occur if the entry is "empty".
    ///              However, if force is set to true, we will clamp anyways.
    void clamp(bool force = false) override
    {
        if (force && EntryModel<T>::empty_)
        {
            set_default();
        }
        else
        {
            EntryModel<T>::clamp(force);
        }
    }

    /// Override base class to do nothing. The boundaries come from
    /// EntryModelBounded::init().
    void set_boundaries() override
    {
    }

    T valueDefault_; ///< default value

    DISALLOW_COPY_AND_ASSIGN(EntryModelBounded);
};

#endif // _UTILS_ENTRYMODEL_HXX_

