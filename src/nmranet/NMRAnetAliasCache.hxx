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
 * \file NMRAnetAliasCache.hxx
 * This file provides an Alias Caching mechanism.
 *
 * @author Stuart W. Baker
 * @date 21 September 2013
 */

#ifndef _NMRAnetAliasCache_hxx_
#define _NMRAnetAliasCache_hxx_

#include <map>

#include "os/OS.hxx"
#include "nmranet/NMRAnetIf.hxx"
#include "utils/macros.h"
#include "utils/RBTree.hxx"
#include "utils/Map.hxx"

namespace NMRAnet
{

/** Cache of alias to node id mappings.  The cache is limited to a fixed number
 * of entries at construction.  All the memory for the cache will be allocated
 * at construction time, limited by the maximum number of entries.  Note, there
 * is no mutual exclusion locking mechanism built into this class.  Mutual
 * exclusion must be handled by the user as needed.
 *
 * @todo the class uses RBTree, consider a version that is a linear search for
 * a small number of entries.
 */
class AliasCache
{
public:
    /** Constructor.
     * @param seed starting seed for generation of aliases
     * @param entries maximum number of entries in this cache
     */
    AliasCache(NodeID seed, size_t entries)
        : freeList(NULL),
          metadata(new Metadata[entries]),
          aliasMap(entries),
          idMap(entries),
          oldest(NULL),
          newest(NULL),
          seed(seed)
    {
        for (size_t i = 0; i < entries; ++i)
        {
            metadata[i].prev = NULL;
            metadata[i].next = freeList;
            freeList = metadata;
        }

        //aliasMap = new AliasMap(nodekey_compare(), Allocator<std::pair<const NodeAlias, Metadata>*>(entries));
    }

    /** Add an alias to an alias cache.
     * @param id 48-bit NMRAnet Node ID to associate alias with
     * @param alias 12-bit alias associated with Node ID
     */
    void add(NodeID id, NodeAlias alias);
    
    /** Remove an alias from an alias cache.
     * @param alias 12-bit alias associated with Node ID
     */
    void remove(NodeAlias alias);

    /** Lookup a node's alias based on its Node ID.
     * @param id Node ID to look for
     * @return alias that matches the Node ID, else 0 if not found
     */
    NodeAlias lookup(NodeID id);

    /** Lookup a node's ID based on its alias.
     * @param alias alias to look for
     * @return Node ID that matches the alias, else 0 if not found
     */
    NodeID lookup(NodeAlias alias);

    /** Call the given callback function once for each alias tracked.
     * @param callback method to call
     * @param context context pointer to pass to callback
     */
    void for_each(void (*callback)(void*, NodeID, NodeAlias), void *context);

    /** Generate a 12-bit pseudo-random alias for a givin alias cache.
     * @return pseudo-random 12-bit alias, an alias of zero is invalid
     */
    NodeAlias generate();

    /** Default destructor */
    ~AliasCache()
    {
        delete[] metadata;
    }
    
private:
    enum
    {
        /** marks an unused mapping */
        UNUSED_MASK = 0x10000000
    };

    struct Metadata
    {
        NodeID id;
        NodeAlias alias;
        long long timestamp;
        union
        {
            Metadata *prev;
            Metadata *newer;
        };
        union
        {
            Metadata *next;
            Metadata *older;
        };
    };

    /** list of unused mapping entries */
    Metadata *freeList;
    Metadata *metadata;

    typedef Map <NodeAlias, Metadata*> AliasMap;
    typedef Map <NodeID, Metadata*> IdMap;

    AliasMap aliasMap;
    IdMap idMap;
    
    Metadata *oldest;
    Metadata *newest;

    /** Seed for the generation of the next alias */
    NodeID seed;
    
    /** Update the time stamp for a given entry.
     * @param  metadata metadata associated with the entry
     */
    void touch(Metadata* metadata)
    {
        metadata->timestamp = OSTime::get_monotonic();

        if (metadata != newest)
        {
            metadata->newer->older = metadata->older;
            metadata->older->newer = metadata->newer;
            metadata->newer = NULL;
            metadata->older = newest;
            newest = metadata;
        }
    }

    /** Default Constructor */
    AliasCache();

    DISALLOW_COPY_AND_ASSIGN(AliasCache);
};

}; /* namepace NMRAnet */

#endif /* _NMRAnetAliasCache_hxx */
