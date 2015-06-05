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
 * \file ConfigRepresentation.hxx
 *
 * Static representation of a config file.
 *
 * @author Balazs Racz
 * @date 31 May 2014
 */

#ifndef _NMRANET_CONFIGREPRESENTATION_HXX_
#define _NMRANET_CONFIGREPRESENTATION_HXX_

namespace nmranet
{

class EndGroup : private ConfigReference
{
public:
    constexpr EndGroup(unsigned offset) : ConfigReference(offset)
    {
    }
    constexpr unsigned size()
    {
        return 0;
    }
};

class Uint16ConfigEntry : public ConfigReference
{
public:
    using ConfigReference::ConfigReference;

    uint16_t get_value(int fd)
    {
        lseek(fd, offset_, SEEK_SET);
        uint16_t value;
        ::read(fd, &value, 2);
        return value;
    }

    static constexpr unsigned size()
    {
        return 2;
    }
};

#define BEGIN_GROUP(group, base)                                               \
    class group##base : public EmptyGroup                                      \
    {                                                                          \
        using EmptyGroup::EmptyGroup;                                          \
    };

class EmptyGroup : public ConfigReference
{
public:
    using ConfigReference::ConfigReference;
    static constexpr unsigned size()
    {
        return 0;
    }
};

#define EXTEND_GROUP(group, prev, name, type)                                  \
    class group##name : public group##prev                                     \
    {                                                                          \
    public:                                                                    \
        using base_type = group##prev;                                         \
        using current_type = type;                                             \
        constexpr group##name(unsigned offset) : base_type(offset)             \
        {                                                                      \
        }                                                                      \
        static constexpr unsigned size()                                       \
        {                                                                      \
            return current_type::size() + offset_from_base();                  \
        }                                                                      \
        static constexpr unsigned offset_from_base()                           \
        {                                                                      \
            return base_type::size();                                          \
        }                                                                      \
        constexpr unsigned offset()                                            \
        {                                                                      \
            return offset_ + offset_from_base();                               \
        }                                                                      \
        constexpr current_type name()                                          \
        {                                                                      \
            return current_type(offset());                                     \
        }                                                                      \
    };

#define END_GROUP(group, prev)                                                 \
    class group : public group##prev                                           \
    {                                                                          \
        using group##prev::group##prev;                                        \
    };

template <class Group, unsigned N> class RepeatedGroup : public ConfigReference
{
public:
    using ConfigReference::ConfigReference;
    static constexpr unsigned size()
    {
        return Group::size() * N;
    }
    constexpr Group entry(unsigned k)
    {
        static_assert(k < N);
        return Group(offset_ + (k * Group::size()));
    }
};

} // namespace nmranet

#endif // _NMRANET_CONFIGREPRESENTATION_HXX_
