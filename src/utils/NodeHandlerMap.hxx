/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file NodeHandlerMap.hxx
 *
 * A reusable data structure for registering global or per-node handlers
 * parametrized by an ID.
 *
 * @author Balazs Racz
 * @date 25 Jan 2014
 */

#ifndef _UTILS_NODEHANDLERMAP_HXX_
#define _UTILS_NODEHANDLERMAP_HXX_

#include <stdint.h>

#include "utils/StlMap.hxx"
#include "utils/LinearMap.hxx"
#include "utils/SysMap.hxx"

/** A map that allows registration and lookup or per-node handler of a
 *  particular message ID.
 *
 *  Regular handlers are registered for a node and messsageID pair.
 *
 *  The map supports registering handlers for a message ID globally by
 *  supplying nullptr as the node. These will be returned for nodes that have
 *  no specific handler for that particular message ID.
 */
class NodeHandlerMapBase
{
private:
    typedef uint64_t key_type;
    typedef void* value_type;

public:
    NodeHandlerMapBase()
    {
        HASSERT(sizeof(void*) == 4);
    }
    
    /// Creates a map with @param entries capacity.
    NodeHandlerMapBase(size_t entries) : entries_(entries)
    {
        HASSERT(sizeof(void*) == 4);
    }

    /** Inserts a handler into the map.
     * @param node is the node for which to register the handler.
     * @param id is the message ID for which to register.
     * @param value is the handler to register.
     */
    void insert(void* node, uint32_t id, void* value)
    {
        entries_[make_key(node, id)] = value;
    }

    /** Finds a handler for a particular node and particular messageID.
     * @returns a handler or nullptr if no node-specific and no globla handler
     * for that ID is found. */
    void* lookup(void* node, uint32_t id)
    {
        auto it = entries_.find(make_key(node, id));
        if (it == entries_.end())
        {
            it = entries_.find(make_key(nullptr, id));
        }
        if (it == entries_.end())
        {
            return nullptr;
        }
        else
        {
            return it->second;
        }
    }

private:
    /// Combines the node pointer and the message ID into a lookup key.
    key_type make_key(void* node, uint32_t id)
    {
        uint64_t key = reinterpret_cast<uint32_t>(node);
        key <<= 32;
        key |= id;
        return key;
    }

    SysMap<key_type, value_type> entries_;
};

template <class Node, class Handler>
class TypedNodeHandlerMap : private NodeHandlerMapBase
{
public:
    TypedNodeHandlerMap()
    {
    }

    TypedNodeHandlerMap(size_t entries) : NodeHandlerMapBase(entries)
    {
    }

    /** Inserts a handler into the map.
     * @param node is the node for which to register the handler.
     * @param id is the message ID for which to register.
     * @param value is the handler to register.
     */
    void insert(Node* node, uint32_t id, Handler* handler)
    {
        NodeHandlerMapBase::insert(node, id, handler);
    }

    /** Finds a handler for a particular node and particular messageID.
     * @returns a handler or nullptr if no node-specific and no globla handler
     * for that ID is found. */
    Handler* lookup(Node* node, uint32_t id)
    {
        return static_cast<Handler*>(NodeHandlerMapBase::lookup(node, id));
    }
};

#endif // _UTILS_NODEHANDLERMAP_HXX_
