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

#include "nmranet/ConfigEntry.hxx"

namespace nmranet
{

#define BEGIN_GROUP(group, base)                                               \
    class group##base : public BaseGroup                                       \
    {                                                                          \
    public:                                                                    \
        using base_type = BaseGroup;                                           \
        using base_type::base_type;                                            \
    };

class BaseGroup : public ConfigReference
{
public:
    using ConfigReference::ConfigReference;
    static constexpr unsigned size()
    {
        return 0;
    }
};

#define EXTEND_GROUP(group, prev_entry_name, entry_name, type, ARGS...)        \
    class group##entry_name : public group##prev_entry_name                    \
    {                                                                          \
    public:                                                                    \
        using base_type = group##prev_entry_name;                              \
        using current_type = type;                                             \
        using base_type::base_type;                                            \
        static constexpr unsigned size()                                       \
        {                                                                      \
            return current_type::size() + offset_from_base();                  \
        }                                                                      \
        static constexpr unsigned offset_from_base()                           \
        {                                                                      \
            return base_type::size();                                          \
        }                                                                      \
        constexpr unsigned last_offset()                                       \
        {                                                                      \
            return offset() + offset_from_base();                              \
        }                                                                      \
        constexpr current_type entry_name()                                    \
        {                                                                      \
            return current_type(last_offset(), ##ARGS);                        \
        }                                                                      \
    };

#define END_GROUP(group, prev_entry_name)                                      \
    class group : public group##prev_entry_name                                \
    {                                                                          \
    public:                                                                    \
        using base_type = group##prev_entry_name;                              \
        using base_type::base_type;                                            \
    };

template <class Group, unsigned N> class RepeatedGroup : public ConfigReference
{
public:
    using ConfigReference::ConfigReference;
    static constexpr unsigned size()
    {
        return Group::size() * N;
    }
    constexpr Group entry(const unsigned k)
    {
        return k < N
            ? Group(offset_ + (k * Group::size()))
            : throw std::logic_error("Tried to fetch an entry of a repeated "
                                     "group that does not exist!");
    }
};

///
/// Defines an empty group with no members, but blocking a certain amount of
/// space in the rendered configuration.
///
template <unsigned N> class EmptyGroup : public ConfigReference
{
public:
    using ConfigReference::ConfigReference;
    static constexpr unsigned size()
    {
        return N;
    }
};

} // namespace nmranet

#endif // _NMRANET_CONFIGREPRESENTATION_HXX_
