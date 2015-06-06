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

#include "utils/StringPrintf.hxx"

namespace nmranet
{

class AtomConfigOptions
{
public:
    struct Name
    {
        constexpr Name(const char *d)
            : d_(d)
        {
        }
        const char *d_;
    };
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

    const char *name = nullptr;
    const char *description = nullptr;
};

class AtomConfigRenderer
{
public:
    enum {
        SKIP_SIZE = 0xffffffff,
    };

    constexpr AtomConfigRenderer(const char *tag, unsigned size)
        : tag_(tag)
        , size_(size)
    {
    }

    std::string render(AtomConfigOptions opts) const
    {
        std::string r = StringPrintf("<%s", tag_);
        if (size_ != SKIP_SIZE) {
            r += StringPrintf(" size=\'%u\'", size_);
        }
        r += ">\n";
        if (opts.name)
        {
            r += StringPrintf("<name>%s</name>\n", opts.name);
        }
        if (opts.description)
        {
            r += StringPrintf(
                "<description>%s</description>\n", opts.description);
        }
        r += StringPrintf("</%s>\n", tag_);
        return r;
    }

private:
    /// XML tag for this atom.
    const char *tag_;
    /// The size attribute of the configuration atom.
    unsigned size_;
};

} // namespace nmranet

#endif // _NMRANET_CONFIGRENDERER_HXX_
