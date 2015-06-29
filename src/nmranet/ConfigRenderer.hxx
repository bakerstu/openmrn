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
 * \file ConfigRenderer.hxx
 *
 * Helper classes for creating a CDI xml file.
 *
 * @author Balazs Racz
 * @date 6 June 2015
 */

#ifndef _NMRANET_CONFIGRENDERER_HXX_
#define _NMRANET_CONFIGRENDERER_HXX_

#include <string>
#include <climits>

#include "utils/StringPrintf.hxx"
#include "nmranet/SimpleNodeInfo.hxx"

namespace nmranet
{

/// Configuration options for rendering CDI (atom) data elements.
class AtomConfigOptions
{
public:
    /// Represent the value enclosed in the <name> tag of the data element.
    struct Name
    {
        constexpr Name(const char *d)
            : d_(d)
        {
        }
        const char *d_;
    };
    /// Represent the value enclosed in the <description> tag of the data
    /// element.
    struct Description
    {
        constexpr Description(const char *d)
            : d_(d)
        {
        }
        const char *d_;
    };

    constexpr AtomConfigOptions()
    {
    }

    template <typename... Args>
    explicit constexpr AtomConfigOptions(const Name n, Args... args)
        : name(n.d_)
        , description(AtomConfigOptions(args...).description)
    {
    }

    template <typename... Args>
    explicit constexpr AtomConfigOptions(const Description d, Args... args)
        : name(AtomConfigOptions(args...).name)
        , description(d.d_)
    {
    }

    void render_cdi(std::string *r) const
    {
        if (name)
        {
            *r += StringPrintf("<name>%s</name>\n", name);
        }
        if (description)
        {
            *r += StringPrintf("<description>%s</description>\n", description);
        }
    }

    const char *name = nullptr;
    const char *description = nullptr;
};

/// Helper class for rendering an atom data element into the cdi.xml.
class AtomConfigRenderer
{
public:
    enum
    {
        SKIP_SIZE = 0xffffffff,
    };

    constexpr AtomConfigRenderer(const char *tag, unsigned size)
        : tag_(tag)
        , size_(size)
    {
    }

    template <typename... Args> void render_cdi(string *s, Args... args) const
    {
        *s += StringPrintf("<%s", tag_);
        if (size_ != SKIP_SIZE)
        {
            *s += StringPrintf(" size=\'%u\'", size_);
        }
        *s += ">\n";
        AtomConfigOptions(args...).render_cdi(s);
        *s += StringPrintf("</%s>\n", tag_);
    }

private:
    /// XML tag for this atom.
    const char *tag_;
    /// The size attribute of the configuration atom.
    unsigned size_;
};

/// Helper class for rendering an empty group of a given size into the cdi.xml.
class EmptyGroupConfigRenderer
{
public:
    enum
    {
        SKIP_SIZE = 0xffffffff,
    };

    constexpr EmptyGroupConfigRenderer(unsigned size)
        : size_(size)
    {
    }

    void render_cdi(string *s) const
    {
        *s += StringPrintf("<group offset='%u'/>", size_);
    }

private:
    /// The number of bytes this group has to skip.
    unsigned size_;
};

/// Configuration options for the CDI group element, as well as representing
/// and distinguishing alternate uses of the BEGIN_GROUP/EXTEND_GROUP/END_GROUP
/// syntax, such as for the toplevel CDI node and for representing segments..
class GroupConfigOptions
{
public:
    using Name = AtomConfigOptions::Name;
    using Description = AtomConfigOptions::Description;
    /// Represents the 'offset' attribute for groups and the 'origin' attribute
    /// for segments.
    struct Offset
    {
        constexpr Offset(int d)
            : d_(d)
        {
        }
        int d_;
    };
    /// Declares that the group is a segment (and thus may be used in the
    /// toplevel CDI.
    struct Segment
    {
        /// @param d is the memory space number (between 0 and 255).
        constexpr Segment(int d)
            : d_(d)
        {
        }
        int d_;
    };

    /// Declares that this group is a toplevel CDI. Causes the group to render
    /// the xml header.
    struct MainCdi
    {
        constexpr MainCdi()
        {
        }
    };

    constexpr GroupConfigOptions()
    {
    }

    template <typename... Args>
    explicit constexpr GroupConfigOptions(const Name n, Args... args)
        : name(n.d_)
        , description(GroupConfigOptions(args...).description)
        , offset(GroupConfigOptions(args...).offset)
        , segment(GroupConfigOptions(args...).segment)
    {
    }

    template <typename... Args>
    explicit constexpr GroupConfigOptions(const Description d, Args... args)
        : name(GroupConfigOptions(args...).name)
        , description(d.d_)
        , offset(GroupConfigOptions(args...).offset)
        , segment(GroupConfigOptions(args...).segment)
    {
    }

    /*
    template <typename... Args>
    explicit constexpr GroupConfigOptions(const XXX d, Args... args)
        : name(GroupConfigOptions(args...).name)
        , description(GroupConfigOptions(args...).description)
        , offset(GroupConfigOptions(args...).offset)
        , segment(GroupConfigOptions(args...).segment)
    {
    */

    template <typename... Args>
    explicit constexpr GroupConfigOptions(const Offset o, Args... args)
        : name(GroupConfigOptions(args...).name)
        , description(GroupConfigOptions(args...).description)
        , offset(o.d_)
        , segment(GroupConfigOptions(args...).segment)
    {
    }

    template <typename... Args>
    explicit constexpr GroupConfigOptions(const Segment s, Args... args)
        : name(GroupConfigOptions(args...).name)
        , description(GroupConfigOptions(args...).description)
        , offset(GroupConfigOptions(args...).offset)
        , segment(s.d_)
    {
    }

    template <typename... Args>
    explicit constexpr GroupConfigOptions(const MainCdi cdi, Args... args)
        : name(GroupConfigOptions(args...).name)
        , description(GroupConfigOptions(args...).description)
        , offset(GroupConfigOptions(args...).offset)
        , segment(-2)
    {
    }

    ///
    /// @return true if this group is a toplevel CDI definition and shall only
    /// allow segments and other toplevel-compatible entries (but no data
    /// elements).
    ///
    constexpr bool is_cdi() const
    {
        return segment == -2;
    }

    ///
    /// @return true if this group is a segment definition.
    ///
    constexpr bool is_segment() const
    {
        return segment >= 0;
    }

    ///
    /// @return the origin of the current segment or zero (default) if not
    /// specified.
    ///
    constexpr unsigned get_segment_offset() const
    {
        return offset == INT_MAX ? 0 : offset;
    }

    void render_cdi(std::string *r) const
    {
        if (name)
        {
            *r += StringPrintf("<name>%s</name>\n", name);
        }
        if (description)
        {
            *r += StringPrintf("<description>%s</description>\n", description);
        }
    }

    const char *name = nullptr;
    const char *description = nullptr;
    int offset = INT_MAX;
    int segment = -1;
};

/// Helper class for rendering the cdi.xml of groups, segments and the toplevel
/// CDI node.
template <class Body> class GroupConfigRenderer
{

public:
    constexpr GroupConfigRenderer(unsigned replication, Body body)
        : replication_(replication)
        , body_(body)
    {
    }

    template <typename... Args> void render_cdi(string *s, Args... args)
    {
        GroupConfigOptions opts(args..., Body::group_opts());
        const char *tag = nullptr;
        *s += "<";
        if (opts.is_cdi())
        {
            *s += "?xml version=\"1.0\"?>\n<";
            tag = "cdi";
            *s += tag;
            *s += " xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" "
                  "xsi:noNamespaceSchemaLocation=\"http://openlcb.org/schema/"
                  "cdi/1/1/cdi.xsd\"";
            HASSERT(replication_ == 1);
            HASSERT(opts.name == nullptr && opts.description == nullptr);
        }
        else if (opts.segment == -1)
        {
            // Regular group
            tag = "group";
            *s += tag;
            if (replication_ != 1)
            {
                *s += StringPrintf(" replication='%u'", replication_);
            }
        }
        else
        {
            // Segment inside CDI.
            tag = "segment";
            *s += tag;
            *s += StringPrintf(" space='%d'", opts.segment);
            if (opts.get_segment_offset() != 0)
            {
                *s += StringPrintf(" origin='%d'", opts.get_segment_offset());
            }
            HASSERT(replication_ == 1);
        }
        *s += ">\n";
        opts.render_cdi(s);
        body_.render_content_cdi(s);
        *s += StringPrintf("</%s>\n", tag);
    }

private:
    /// For regular groups, the count of replicas.
    unsigned replication_;
    /// Object representing the contents of this group. Must have a
    /// render_content_cdi() call.
    Body body_;
};

/// Helper class for rendering the <identification> tag.
class IdentificationRenderer
{
public:
    constexpr IdentificationRenderer()
    {
    }

    static void render_tag(const char *tag, const char *value, string *s)
    {
        *s += StringPrintf("<%s>%s</%s>\n", tag, value, tag);
    }

    void render_cdi(string *s) const
    {
        extern const SimpleNodeStaticValues SNIP_STATIC_DATA;
        *s += "<identification>\n";
        render_tag("manufacturer", SNIP_STATIC_DATA.manufacturer_name, s);
        render_tag("model", SNIP_STATIC_DATA.model_name, s);
        render_tag("hardwareVersion", SNIP_STATIC_DATA.hardware_version, s);
        render_tag("softwareVersion", SNIP_STATIC_DATA.software_version, s);
        *s += "</identification>\n";
    }
};

/// Helper class for rendering the <acdi> tag.
class AcdiRenderer
{
public:
    constexpr AcdiRenderer()
    {
    }

    void render_cdi(string *s) const
    {
        s->append("<acdi/>\n");
    }
};

} // namespace nmranet

#endif // _NMRANET_CONFIGRENDERER_HXX_
