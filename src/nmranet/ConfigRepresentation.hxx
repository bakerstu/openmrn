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

#define BEGIN_GROUP(group, base, ARGS...)                                      \
    class group##base : public nmranet::BaseGroup                              \
    {                                                                          \
    public:                                                                    \
        using base_type = BaseGroup;                                           \
        using base_type::base_type;                                            \
        using Name = AtomConfigOptions::Name;                                  \
        using Description = AtomConfigOptions::Description;                    \
        using Segment = GroupConfigOptions::Segment;                           \
        using Offset = GroupConfigOptions::Offset;                             \
        using MainCdi = GroupConfigOptions::MainCdi;                           \
        static constexpr GroupConfigOptions group_opts()                       \
        {                                                                      \
            return GroupConfigOptions(ARGS);                                   \
        }                                                                      \
        void render_cdi(std::string *s) const                                  \
        {                                                                      \
        }                                                                      \
    };

class BaseGroup : public nmranet::ConfigReference
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
        using Name = AtomConfigOptions::Name;                                  \
        using Description = AtomConfigOptions::Description;                    \
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
            static_assert(!group_opts().is_cdi() ||                            \
                    current_type(0).group_opts().is_segment(),                 \
                "May only have segments inside CDI.");                         \
            return group_opts().is_cdi()                                       \
                ? current_type(                                                \
                      current_type(0).group_opts().get_segment_offset())       \
                : current_type(last_offset());                                 \
        }                                                                      \
        void render_cdi(std::string *s) const                                  \
        {                                                                      \
            base_type::render_cdi(s);                                          \
            entry_name().config_renderer().render_cdi(s, ##ARGS);              \
        }                                                                      \
    };

#define END_GROUP(group, prev_entry_name)                                      \
    class group : public group##prev_entry_name                                \
    {                                                                          \
    public:                                                                    \
        using base_type = group##prev_entry_name;                              \
        using base_type::base_type;                                            \
        void render_content_cdi(std::string *s) const                          \
        {                                                                      \
            base_type::render_cdi(s);                                          \
        }                                                                      \
        constexpr GroupConfigRenderer<group> config_renderer()                 \
        {                                                                      \
            return GroupConfigRenderer<group>(1, *this);                       \
        }                                                                      \
    };

template <class Group, unsigned N> class RepeatedGroup : public ConfigEntryBase
{
public:
    using base_type = ConfigEntryBase;
    using base_type::base_type;
    static constexpr unsigned size()
    {
        return Group::size() * N;
    }
    template <int K> constexpr Group entry()
    {
        static_assert(K < N, "Tried to fetch an entry of a repeated "
                             "group that does not exist!");
        return Group(offset_ + (K * Group::size()));
    }
    constexpr GroupConfigRenderer<Group> config_renderer()
    {
        return GroupConfigRenderer<Group>(N, entry<0>());
    }
};

///
/// Defines an empty group with no members, but blocking a certain amount of
/// space in the rendered configuration.
///
template <unsigned N> class EmptyGroup : public ConfigEntryBase
{
public:
    using base_type = ConfigEntryBase;
    using base_type::base_type;
    static constexpr unsigned size()
    {
        return N;
    }
    constexpr EmptyGroupConfigRenderer config_renderer()
    {
        return EmptyGroupConfigRenderer(N);
    }
};

class ToplevelEntryBase : public ConfigEntryBase
{
public:
    using base_type = ConfigEntryBase;
    using base_type::base_type;
    static constexpr GroupConfigOptions group_opts()
    {
        return GroupConfigOptions(GroupConfigOptions::Segment(1000));
    }
    static constexpr unsigned size()
    {
        return 0;
    }
};

class Identification : public ToplevelEntryBase
{
public:
    using base_type = ToplevelEntryBase;
    using base_type::base_type;
    static constexpr IdentificationRenderer config_renderer()
    {
        return IdentificationRenderer();
    }
};

} // namespace nmranet

#endif // _NMRANET_CONFIGREPRESENTATION_HXX_
