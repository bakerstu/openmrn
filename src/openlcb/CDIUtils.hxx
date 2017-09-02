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
    /// Helper class for unique_ptr to delete an XML document correctly.
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

    /// Smart pointer class for holding XML documents.
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

    /// Finds the name of a CDI element.
    ///
    /// @param node is a CDI element (segment, group, or config element).
    /// @return finds a child tag <name> and returns its contents. Returns
    /// empty string if no <name> was found.
    static string find_node_name(XMLNode *node, const char *def)
    {
        auto *n = find_child_or_null(node, "name");
        if (n == nullptr || n->text == nullptr)
            return def;
        return n->text;
    }

    /// Find the description of a CDI element.
    ///
    /// @param node is a CDI element (segment, group, or config element).
    /// @return finds a child tag <description> and returns its
    /// contents. Returns empty string if no <description> was found.
    static string find_node_description(XMLNode *node)
    {
        auto *n = find_child_or_null(node, "description");
        if (n == nullptr || n->text == nullptr)
            return "";
        return n->text;
    }

    /// Clears out al luserinfo structure pointers. This is necessary to use
    /// the new_userinfo call below. Call this after the XML has been
    /// successfully parsed.
    /// @param doc document to prepare up.
    static void prepare_doc(XMLDoc *doc)
    {
        auto *node = XMLDoc_root(doc);
        while (node)
        {
            node->user = nullptr;
            node = XMLNode_next(node);
        }
    }

    /// Deletes all userinfo structures allocated in a doc.
    /// @param doc document to clean up.
    static void cleanup_doc(XMLDoc *doc)
    {
        auto *node = XMLDoc_root(doc);
        while (node)
        {
            free(node->user);
            node = XMLNode_next(node);
        }
    }

    /// Allocates a new object of type T for the node as userinfo; calls T's
    /// constructor with args..., stores the resulting pointer in the userinfo
    /// pointer of node.
    /// @param info output argument for the userinfo structure pointer.
    /// @param node which XML node the userinfo should point at.
    /// @param args... forwarded as the constructor arguments for T (can be
    /// empty).
    template <class T, typename... Args>
    static void new_userinfo(T **info, XMLNode *node, Args &&... args)
    {
        HASSERT(node->user == nullptr);
        static_assert(std::is_trivially_destructible<T>::value == true,
            "Userdata attached to nodes must be trivially destructible");
        *info = static_cast<T *>(malloc(sizeof(T)));
        new (*info) T(std::forward<Args>(args)...);
        node->user = *info;
    }

    /// Retrieve the userinfo structure from an XML node.
    /// @param info will be set to the userinfo structure using an unchecked
    /// cast to T. This variable must be of the same (or compatible) type as
    /// what the userinfo has been allocated to.
    /// @param node is the XML element node whose userinfo we are trying to
    /// fetch
    template <class T> static void get_userinfo(T **info, XMLNode *node)
    {
        HASSERT(node);
        *info = static_cast<T *>(node->user);
    }

    /// Allocation data we hold about a Data Element in its userinfo structure.
    struct NodeInfo
    {
        /// Offset of the address of this element from the address of the
        /// parent group element. This is the sum of size values of the
        /// preceding elements within the given group. Inside a repeated group
        /// these offsets are counted from the current repetition start offset.
        int offset_from_parent = 0;
        /// Total number of bytes that this element occupies. This includes all
        /// repetitions for a repeated group.
        int size = 0;
    };

    static int get_numeric_attribute(
        XMLNode *node, const char *attr_name, int def = 0)
    {
        const SXML_CHAR *attr_value;
        XMLNode_get_attribute_with_default(
            node, attr_name, &attr_value, nullptr);
        if ((!attr_value) || (attr_value[0] == 0))
        {
            return def;
        }
        return atoi(attr_value);
    }

    /// Allocates all userinfo structures within a segment and performs the
    /// offset layout algorithm.
    static void layout_segment(XMLNode *segment)
    {
        HASSERT(strcmp(segment->tag, "segment") == 0);
        unsigned current_offset = get_numeric_attribute(segment, "origin");
        NodeInfo *info;
        new_userinfo(&info, segment);
        info->offset_from_parent = current_offset;
        if (XMLNode_get_children_count(segment) == 0)
            return;
        XMLNode *current_parent = segment;
        XMLNode *current_child = XMLNode_get_child(segment, 0);
        NodeInfo *parent_info = info;
        while (true)
        {
            if (strcmp(current_child->tag, "name") == 0 ||
                strcmp(current_child->tag, "description") == 0 ||
                strcmp(current_child->tag, "repname") == 0)
            {
                // Do nothing, not a data element
            }
            else
            {
                new_userinfo(&info, current_child);
                parent_info->size +=
                    get_numeric_attribute(current_child, "offset", 0);
                info->offset_from_parent = parent_info->size;
                if (strcmp(current_child->tag, "eventid") == 0)
                {
                    info->size = 8;
                }
                else if (strcmp(current_child->tag, "string") == 0 ||
                    strcmp(current_child->tag, "int") == 0 ||
                    strcmp(current_child->tag, "float") == 0)
                {
                    info->size =
                        get_numeric_attribute(current_child, "size", 1);
                }
                else if (strcmp(current_child->tag, "group") == 0)
                {
                    if (XMLNode_get_children_count(current_child) > 0)
                    {
                        current_parent = current_child;
                        current_child = XMLNode_get_child(current_parent, 0);
                        get_userinfo(&parent_info, current_parent);
                        continue;
                    }
                    // an empty group has size == 0 and we don't need to do
                    // anything here.
                }
                parent_info->size += info->size;
            }

            // Move to next child.
            while ((current_child = XMLNode_next_sibling(current_child)) ==
                nullptr)
            {
                // End of children; must go up.
                if (current_parent == segment)
                {
                    // nowhere to go up
                    break;
                }
                current_child = current_parent;
                current_parent = current_child->father;
                // handle groups with repetitions
                get_userinfo(&info, current_child);
                get_userinfo(&parent_info, current_parent);
                int repcount =
                    get_numeric_attribute(current_child, "replication", 1);
                info->size *= repcount;
                parent_info->size += info->size;
            }
            if (current_parent == segment && current_child == nullptr)
            {
                // end of iteration
                break;
            }
        }
    }

private:
    // Static class; never instantiated.
    CDIUtils();
};

} // namespace openlcb

#endif // _OPENLCB_CDIUTILS_HXX_
