/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file AliasCache.cxx
 * This file provides an Alias Caching mechanism.
 *
 * @author Stuart W. Baker
 * @date 21 September 2013
 */

#include "openlcb/AliasCache.hxx"

#include <set>

#include "os/OS.hxx"
#include "utils/logging.h"

#ifdef GTEST
#define TEST_CONSISTENCY
#endif

namespace openlcb
{

#define CONSTANT 0x1B0CA37ABA9 /**< constant for random number generation */

/// This code removes the unique bits in the stored node alias in case this is
/// a NOT_RESPONDING entry.
/// @param stored alias in the metadata storage
/// @return the alias if it's valid or NOT_RESPONDING ifthis is a sentinel
static NodeAlias resolve_notresponding(NodeAlias stored)
{
    if ((stored & NOT_RESPONDING) == NOT_RESPONDING)
    {
        return NOT_RESPONDING;
    }
    return stored;
}

#if defined(TEST_CONSISTENCY)
extern volatile int consistency_result;
volatile int consistency_result = 0;

int AliasCache::check_consistency()
{
    if (idMap.size() != aliasMap.size())
    {
        LOG(INFO, "idmap size != aliasmap size.");
        return 1;
    }
    if (aliasMap.size() == entries)
    {
        if (!freeList.empty())
        {
            LOG(INFO, "Found freelist entry when map is full.");
            return 2;
        }
    }
    else
    {
        if (freeList.empty())
        {
            LOG(INFO, "No freelist entry although map is not full.");
            return 3;
        }
    }
    if (aliasMap.size() == 0 && (!oldest.empty() || !newest.empty()))
    {
        LOG(INFO, "LRU head/tail elements should be null when map is empty.");
        return 4;
    }
    std::set<void *> free_entries;
    for (PoolIdx p = freeList; !p.empty(); p = p.deref(this)->older_)
    {
        Metadata *m = p.deref(this);
        if (free_entries.count(m))
        {
            LOG(INFO, "Duplicate entry on freelist.");
            return 5;
        }
        free_entries.insert(m);
    }
    if (free_entries.size() + aliasMap.size() != entries)
    {
        LOG(INFO, "Lost some metadata entries.");
        return 6;
    }
    for (auto kv : aliasMap)
    {
        if (free_entries.count(kv.deref(this)))
        {
            LOG(INFO, "Found an aliasmap entry in the freelist.");
            return 19;
        }
    }
    for (auto kv : idMap)
    {
        if (free_entries.count(kv.deref(this)))
        {
            LOG(INFO, "Found an id entry in the freelist.");
            return 20;
        }
    }
    if (aliasMap.size() == 0)
    {
        if (!oldest.empty())
        {
            LOG(INFO, "Oldest should be empty when map is empty.");
            return 7;
        }
        if (!newest.empty())
        {
            LOG(INFO, "Newest should be empty when map is empty.");
            return 8;
        }
    }
    else
    {
        if (oldest.empty())
        {
            LOG(INFO, "Oldest should be nonempty when map is nonempty.");
            return 9;
        }
        if (newest.empty())
        {
            LOG(INFO, "Newest should be nonempty when map is nonempty.");
            return 10;
        }
        if (free_entries.count(oldest.deref(this)))
        {
            LOG(INFO, "Oldest is on the freelist.");
            return 11; // oldest is free
        }
        if (free_entries.count(newest.deref(this)))
        {
            LOG(INFO, "Newest is on the freelist.");
            return 12; // newest is free
        }
    }
    if (aliasMap.size() == 0)
    {
        return 0;
    }
    // Check linking.
    {
        PoolIdx prev = oldest;
        unsigned count = 1;
        if (!prev.deref(this)->older_.empty())
        {
            LOG(INFO, "Prev link points to empty.");
            return 13;
        }
        while (!prev.deref(this)->newer_.empty())
        {
            auto next = prev.deref(this)->newer_;
            ++count;
            if (free_entries.count(next.deref(this)))
            {
                LOG(INFO, "Next link points to the freelist.");
                return 21;
            }
            if (next.deref(this)->older_.idx_ != prev.idx_)
            {
                LOG(INFO, "Next link broken.");
                return 14;
            }
            prev = next;
        }
        if (prev.idx_ != newest.idx_)
        {
            LOG(INFO, "Prev link points to newest.");
            return 18;
        }
        if (count != aliasMap.size())
        {
            LOG(INFO, "LRU link list length is incorrect.");
            return 27;
        }
    }
    {
        PoolIdx next = newest;
        if (!next.deref(this)->newer_.empty())
        {
            LOG(INFO, "Newest has a newer link.");
            return 15;
        }
        while (!next.deref(this)->older_.empty())
        {
            auto prev = next.deref(this)->older_;
            if (free_entries.count(prev.deref(this)))
            {
                LOG(INFO, "Prev link points to the freelist.");
                return 22;
            }
            if (prev.deref(this)->newer_.idx_ != next.idx_)
            {
                LOG(INFO, "Prev link broken.");
                return 16;
            }
            next = prev;
        }
        if (next.idx_ != oldest.idx_)
        {
            LOG(INFO, "Next link points to oldest.");
            return 17;
        }
    }
    for (unsigned i = 0; i < entries; ++i)
    {
        if (free_entries.count(pool + i))
        {
            continue;
        }
        auto *e = pool + i;
        if (idMap.find(e->get_node_id()) == idMap.end())
        {
            LOG(INFO, "Metadata ID is not in the id map.");
            return 23;
        }
        if (idMap.find(e->get_node_id())->idx_ != i)
        {
            LOG(INFO,
                "Id map entry does not point back to the expected index.");
            return 24;
        }
        if (aliasMap.find(e->alias_) == aliasMap.end())
        {
            LOG(INFO, "Metadata alias is not in the alias map.");
            return 25;
        }
        if (aliasMap.find(e->alias_)->idx_ != i)
        {
            LOG(INFO,
                "Alis map entry does not point back to the expected index.");
            return 26;
        }
    }
    return 0;
}

#endif

void AliasCache::clear()
{
    idMap.clear();
    aliasMap.clear();
    oldest.idx_ = NONE_ENTRY;
    newest.idx_ = NONE_ENTRY;
    freeList.idx_ = NONE_ENTRY;
    /* initialize the freeList */
    for (size_t i = 0; i < entries; ++i)
    {
        pool[i].newer_.idx_ = NONE_ENTRY;
        pool[i].older_ = freeList;
        freeList.idx_ = i;
    }
}

void debug_print_entry(void *, NodeID id, NodeAlias alias)
{
    LOG(INFO, "[%012" PRIx64 "]: %03X", id, alias);
}

void debug_print_cache(AliasCache *c)
{
    LOG(INFO, "Alias cache:");
    c->for_each(&debug_print_entry, nullptr);
}

/** Add an alias to an alias cache.
 * @param id 48-bit NMRAnet Node ID to associate alias with
 * @param alias 12-bit alias associated with Node ID
 */
void AliasCache::add(NodeID id, NodeAlias alias)
{
    HASSERT(id != 0);
    HASSERT(alias != 0);
    
    Metadata *insert;

    auto it = aliasMap.find(alias);
    if (alias == NOT_RESPONDING)
    {
        // We can have more than one NOT_RESPONDING entry.
        it = aliasMap.end();
    }
    if (it != aliasMap.end())
    {
        /* we already have a mapping for this alias, so lets remove it */
        insert = it->deref(this);
        auto nid = insert->get_node_id();
        remove(insert->alias_);

        if (removeCallback)
        {
            /* tell the interface layer that we removed this mapping */
            (*removeCallback)(nid, insert->alias_, context);
        }
    }
    auto nit = idMap.find(id);
    if (nit != idMap.end())
    {
        /* we already have a mapping for this id, so lets remove it */
        insert = nit->deref(this);
        auto nid = insert->get_node_id();
        remove(insert->alias_);

        if (removeCallback)
        {
            /* tell the interface layer that we removed this mapping */
            (*removeCallback)(nid, insert->alias_, context);
        }
    }

    if (!freeList.empty())
    {
        /* found an empty slot */
        insert = freeList.deref(this);
        freeList = insert->older_;
    }
    else
    {
        HASSERT(!oldest.empty() && !newest.empty());

        /* kick out the oldest mapping and re-link the oldest endpoint */
        insert = oldest.deref(this);
        auto second = insert->newer_;
        if (!second.empty())
        {
            second.deref(this)->older_.idx_ = NONE_ENTRY;
        }
        if (insert == newest.deref(this))
        {
            newest.idx_ = NONE_ENTRY;
        }
        oldest = second;

        aliasMap.erase(aliasMap.find(insert->alias_));
        idMap.erase(idMap.find(insert->get_node_id()));

        if (removeCallback)
        {
            /* tell the interface layer that we removed this mapping */
            (*removeCallback)(insert->get_node_id(), insert->alias_, context);
        }
    }

    if (alias == NOT_RESPONDING)
    {
        // This code will make all NOT_RESPONDING aliases unique in our map.
        unsigned ofs = insert - pool;
        alias = NOT_RESPONDING | ofs;
        auto it = aliasMap.find(alias);
        HASSERT(it == aliasMap.end());
    }
    insert->set_node_id(id);
    insert->alias_ = alias;

    PoolIdx n;
    n.idx_ = insert - pool;
    aliasMap.insert(PoolIdx(n));
    idMap.insert(PoolIdx(n));

    /* update the time based list */
    insert->newer_.idx_ = NONE_ENTRY;
    if (newest.empty())
    {
        /* if newest == NULL, then oldest must also be NULL */
        HASSERT(oldest.empty());

        insert->older_.idx_ = NONE_ENTRY;
        oldest = n;
    }
    else
    {
        insert->older_ = newest;
        newest.deref(this)->newer_ = n;
    }

    newest = n;

#if defined(TEST_CONSISTENCY)
    consistency_result = check_consistency();
    HASSERT(0 == consistency_result);
#endif
}

/** Remove an alias from an alias cache.  This method does not call the
 * remove_callback method passed in at construction since it is a
 * deliberate call not requiring notification.
 * @param alias 12-bit alias associated with Node ID
 */
void AliasCache::remove(NodeAlias alias)
{
    auto it = aliasMap.find(alias);

    if (it != aliasMap.end())
    {
        Metadata *metadata = it->deref(this);
        aliasMap.erase(it);
        idMap.erase(idMap.find(metadata->get_node_id()));
        // Ensures that the AME query handler does not find this metadata.
        metadata->set_node_id(0);

        // Removes metadata from the linked lists.
        if (!metadata->newer_.empty())
        {
            metadata->newer_.deref(this)->older_ = metadata->older_;
        }
        if (!metadata->older_.empty())
        {
            metadata->older_.deref(this)->newer_ = metadata->newer_;
        }
        if (metadata == newest.deref(this))
        {
            newest = metadata->older_;
        }
        if (metadata == oldest.deref(this))
        {
            oldest = metadata->newer_;
        }

        // Adds metadata to the freelist.
        metadata->older_ = freeList;
        freeList.idx_ = metadata - pool;
    }

#if defined(TEST_CONSISTENCY)
    consistency_result = check_consistency();
    HASSERT(0 == consistency_result);
#endif
}

bool AliasCache::retrieve(unsigned entry, NodeID* node, NodeAlias* alias)
{
    HASSERT(entry < size());
    Metadata* md = pool + entry;
    if (!md->alias_)
    {
        return false;
    }
    if (node)
    {
        *node = md->get_node_id();
    }
    if (alias)
    {
        *alias = resolve_notresponding(md->alias_);
    }
    return true;
}

bool AliasCache::next_entry(NodeID bound, NodeID *node, NodeAlias *alias)
{
    auto it = idMap.upper_bound(bound);
    if (it == idMap.end())
    {
        return false;
    }
    Metadata *metadata = it->deref(this);
    if (alias)
    {
        *alias = resolve_notresponding(metadata->alias_);
    }
    if (node)
    {
        *node = metadata->get_node_id();
    }
    return true;
}

/** Lookup a node's alias based on its Node ID.
 * @param id Node ID to look for
 * @return alias that matches the Node ID, else 0 if not found
 */
NodeAlias AliasCache::lookup(NodeID id)
{
    HASSERT(id != 0);

    auto it = idMap.find(id);

    if (it != idMap.end())
    {
        Metadata *metadata = it->deref(this);

        /* update timestamp */
        touch(metadata);
        return resolve_notresponding(metadata->alias_);
    }

    /* no match found */
    return 0;
}

/** Lookup a node's ID based on its alias.
 * @param alias alias to look for
 * @return Node ID that matches the alias, else 0 if not found
 */
NodeID AliasCache::lookup(NodeAlias alias)
{
    if (alias == 0)
    {
        return 0;
    }

    auto it = aliasMap.find(alias);

    if (it != aliasMap.end())
    {
        Metadata *metadata = it->deref(this);

        /* update timestamp */
        touch(metadata);
        return metadata->get_node_id();
    }
    
    /* no match found */
    return 0;
}

/** Call the given callback function once for each alias tracked.  The order
 * will be in last "touched" order.
 * @param callback method to call
 * @param context context pointer to pass to callback
 */
void AliasCache::for_each(void (*callback)(void*, NodeID, NodeAlias), void *context)
{
    HASSERT(callback != NULL);

    for (PoolIdx idx = newest; !idx.empty(); idx = idx.deref(this)->older_)
    {
        Metadata *metadata = idx.deref(this);
        (*callback)(context, metadata->get_node_id(),
            resolve_notresponding(metadata->alias_));
    }
}

/** Generate a 12-bit pseudo-random alias for a givin alias cache.
 * @return pseudo-random 12-bit alias, an alias of zero is invalid
 */
NodeAlias AliasCache::generate()
{
    NodeAlias alias;

    do
    {
        /* calculate the alias given the current seed */
        alias = (seed ^ (seed >> 12) ^ (seed >> 24) ^ (seed >> 36)) & 0xfff;

        /* calculate the next seed */
        seed = ((((1 << 9) + 1) * (seed) + CONSTANT)) & 0xffffffffffff;
    } while (alias == 0 || lookup(alias) != 0);

    /* new random alias */
    return alias;
}

/** Update the time stamp for a given entry.
 * @param  metadata metadata associated with the entry
 */
void AliasCache::touch(Metadata* metadata)
{
    if (metadata != newest.deref(this))
    {
        if (metadata == oldest.deref(this))
        {
            oldest = metadata->newer_;
            oldest.deref(this)->older_.idx_ = NONE_ENTRY;
        }
        else
        {
            /* we have someone older */
            metadata->older_.deref(this)->newer_ = metadata->newer_;
        }
        metadata->newer_.deref(this)->older_ = metadata->older_;
        metadata->newer_.idx_ = NONE_ENTRY;
        metadata->older_ = newest;
        newest.deref(this)->newer_.idx_ = metadata - pool;
        newest.idx_ = metadata - pool;
    }
#if defined(TEST_CONSISTENCY)
    consistency_result = check_consistency();
    HASSERT(0 == consistency_result);
#endif
}

}
