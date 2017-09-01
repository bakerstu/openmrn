/** \copyright
 * Copyright (c) 2017, Balazs Racz
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
 * \file CDIUtils.hxx
 *
 * Utility library for interpreting CDI XML files.
 *
 * @author Balazs Racz
 * @date 31 Aug 2017
 */

#ifndef _OPENLCB_CDIUTILS_HXX_
#define _OPENLCB_CDIUTILS_HXX_

#include "sxmlc.h"

namespace openlcb
{

class CDIUtils
{
public:
    struct XMLDocDeleter
    {
        void operator()(XMLDoc *doc)
        {
            if (!doc)
                return;
            cleanup_doc(doc);
            XMLDoc_free(doc);
            delete doc;
        }
    };

    typedef std::unique_ptr<XMLDoc, XMLDocDeleter> xmldoc_ptr_t;
    
    /// Searches the list of children for the first child with a specific tag.
    /// @param parent is the node whose children to search.
    /// @param tag is which child to look for.
    /// @return nullptr if not found, else the first occurrence of tag in the
    /// children.
    static XMLNode *find_child_or_null(XMLNode *parent, const char *tag)
    {
        auto count = XMLNode_get_children_count(parent);
        for (int i = 0; i < count; ++i)
        {
            auto *c = XMLNode_get_child(parent, i);
            if (strcmp(tag, c->tag) == 0)
                return c;
        }
        return nullptr;
    }

    static string find_node_name(XMLNode *node, const char *def)
    {
        auto *n = find_child_or_null(node, "name");
        if (n == nullptr || n->text == nullptr)
            return def;
        return n->text;
    }

    static string find_node_description(XMLNode *node)
    {
        auto *n = find_child_or_null(node, "description");
        if (n == nullptr || n->text == nullptr)
            return "";
        return n->text;
    }

    static void prepare_doc(XMLDoc *doc)
    {
        auto *node = XMLDoc_root(doc);
        while (node)
        {
            node->user = nullptr;
            node = XMLNode_next(node);
        }
    }

    static void cleanup_doc(XMLDoc *doc)
    {
        auto *node = XMLDoc_root(doc);
        while (node)
        {
            free(node->user);
            node = XMLNode_next(node);
        }
    }

    template <class T, typename... Args>
    static void new_userinfo(T **info, XMLNode *node, Args &&... args)
    {
        HASSERT(node->user == nullptr);
        static_assert(std::is_trivially_destructible<T>::value == true,
            "Userdata attached to nodes must be trivially destructible");
        *info = malloc(sizeof(T));
        new (*info) T(std::forward<Args>(args)...);
        node->user = *info;
    }

private:
    // Static class; never instantiated.
    CDIUtils();
};

} // namespace openlcb

#endif // _OPENLCB_CDIUTILS_HXX_
