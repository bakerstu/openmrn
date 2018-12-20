/** @copyright
 * Copyright (c) 2017, Stuart W. Baker
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without written consent.
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
#include <type_traits>

#include "utils/format_utils.hxx"

/** Implementation of a text entry menu.
 * @tparam T the data type up to 64-bits in size
 * @tparam N the size of the entry in max number of visible digits.
 */
template <class T, size_t N> class EntryModel
{
public:
    /** Constructor.
     * @param transform force characters to be upper case
     */
    EntryModel(bool transform = false)
        : digits_(0)
        , index_(0)
        , hasInitial_(false)
        , transform_(transform)
        , base_(10)
    {
        clear();
        data_[N] = '\0';
    }

    /** Initialize empty.
     * @param digits max number of significant digits in the base type
     * @param base base type, 10 or 16
     */
    void init(unsigned digits, int base)
    {
        HASSERT(digits > 0 && digits <= N);
        digits_ = digits;
        clear();
        hasInitial_ = true;
    }

    /** Initialize with a value.
     * @param digits max number of significant digits in the base type
     * @param base base type, 10 or 16
     * @param value value to initialize with
     */
    void init(unsigned digits, int base, T value)
    {
        HASSERT(digits > 0 && digits <= N);
        digits_ = digits;
        clear();

        string str;
        switch (base)
        {
            default:
                HASSERT(0);
            case 10:
                if (std::is_signed<T>::value)
                {
                    str = int64_to_string(value, digits);
                }
                else
                {
                    str = uint64_to_string(value, digits);
                }
                break;
            case 16:
                if (std::is_signed<T>::value)
                {
                    str = int64_to_string_hex(value, digits);
                }
                else
                {
                    str = uint64_to_string_hex(value, digits);
                }
                if (transform_)
                {
                    /* equires all characters in upper case */
                    transform(str.begin(), str.end(), str.begin(), toupper);
                }
                break;
        }
        strncpy(data_, str.c_str(), sizeof(data_) - 1);
        data_[sizeof(data_) - 1] = '\0';
        hasInitial_ = true;
    }

    /** Clear the entry string.
     * @param data data to fill in the buffer with
     */
    void clear(const char *data = nullptr)
    {
        memset(data_, ' ', digits_);
        if (data)
        {
            memcpy(data_, data, strlen(data));
        }
        data_[digits_] = '\0';
        index_ = 0;
    }

    /** Get the current index.
     * @return current cursor index
     */
    unsigned cursor_index()
    {
        return index_;
    }

    /** Test if cursor is visible.
     * @return true if cursor is visiable, else false
     */
    bool cursor_visible()
    {
        return index_ < digits_;
    }

    /** Put a character at the current index, and increment the index by 1.
     * @param c Character to place
     * @return true if the string was cleared out and an LCD refresh is
     *         may be required.
     */
    bool putc_inc(char c)
    {
        bool refresh = false;
        if (index_ >= digits_)
        {
            refresh = true;
            clear();
        }
        else
        {
            if (hasInitial_)
            {
                hasInitial_ = false;
                refresh = true;
                clear();
            }
            data_[index_++] = transform_ ? toupper(c) : c;
        }
        clamp();
        return refresh;
    }

    /** Delete a character off the end.
     */
    void backspace()
    {
        if (index_ > 0)
        {
            data_[--index_] = ' ';
            hasInitial_ = false;
        }
        clamp();
    }

    /** Set the radix base.
     * @param base new radix base to set.
     */
    void set_base(int base)
    {
        HASSERT(base == 10 || base == 16);
        base_ = base;
    }

    /** Set the value, keep the digits and base the same.
     * @param value value to initialize with
     */
    void set_value(T value)
    {
        init(digits_, base_, value);
    }

    /** Get the entry as an unsigned integer value.
     * @param start_index starting index in string to start conversion
     * @return value representation of the string
     */
    T get_value(unsigned start_index = 0)
    {
        if (std::is_signed<T>::value)
        {
            return strtoll(data_ + start_index, NULL, base_);
        }
        else
        {
            return strtoull(data_ + start_index, NULL, base_);
        }
    }

    /** Get the C style string representing the menu entry.
     * @return the string data representing the menu entry
     */
    const char *c_str()
    {
        return data_;
    }

    /** Get a copy of the string without any whitespace.
     * @param strip_leading true to also strip off leading '0' or ' '
     * @return the string data representing the menu entry
     */
    string parsed(bool strip_leading = false)
    {
        const char *parse = data_;
        string result;
        result.reserve(N);
        if (strip_leading)
        {
            while (*parse == '0' || *parse == ' ')
            {
                ++parse;
            }
        }
        while (*parse != ' ' && *parse != '\0')
        {
            result.push_back(*parse++);
        }
        return result;
    }

    /** Copy the entry data into the middle of a buffer.
     * @param buf pointer to destination buffer
     * @param start_index starting index of buffer for the copy destination
     * @param digits number of digits to copy
     */
    void copy_to_buffer(char *buf, int start_index, int digits)
    {
        HASSERT(digits <= digits_);
        memcpy(buf + start_index, data_, digits);
    }

    /** Change the sign of the data.
     */
    void change_sign()
    {
        HASSERT(std::is_signed<T>::value);
        if (hasInitial_)
        {
            set_value(-get_value());
        }
        else if (data_[0] == '-')
        {
            memmove(data_, data_ + 1, digits_ - 1);
            data_[digits_] = ' ';
        }
        else
        {
            memmove(data_ + 1, data_, digits_ - 1);
            data_[0] = '-';
        }
        clamp();
    }

protected:
    /// Clamp the value at the min or max.
    virtual void clamp()
    {
    }

private:
    unsigned digits_     : 5; /**< number of significant digits */
    unsigned index_      : 5; /**< present write index */
    unsigned hasInitial_ : 1; /**< has an initial value */
    unsigned transform_  : 1; /**< force characters to be upper case */
    unsigned reserved_   : 20; /**< reserved bit space */

    int base_; /**< radix base */
    char data_[N + 1]; /**< data string */

    DISALLOW_COPY_AND_ASSIGN(EntryModel);
};

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
        : EntryModel<T, N>(transform)
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

private:
    /// Clamp the value at the min or max.
    void clamp() override
    {
        T value = EntryModel<T, N>::get_value();
        if (value < min_)
        {
            EntryModel<T, N>::set_value(min_);
        }
        else if (value > max_)
        {
            EntryModel<T, N>::set_value(max_);
        }
    }

    T min_; ///< minimum value
    T max_; ///< maximum value
    T default_; ///< default value

    DISALLOW_COPY_AND_ASSIGN(EntryModelBounded);
};

#endif /* _UTILS_ENTRYMODEL_HXX_ */

