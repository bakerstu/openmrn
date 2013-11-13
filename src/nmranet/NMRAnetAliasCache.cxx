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
 * \file NMRAnetAliasCache.cxx
 * This file provides an Alias Caching mechanism.
 *
 * @author Stuart W. Baker
 * @date 21 September 2013
 */

#include "nmranet/NMRAnetAliasCache.hxx"

namespace NMRAnet
{

#define CONSTANT 0x1B0CA37ABA9 /**< constant for random number generation */

/** Add an alias to an alias cache.
 * @param id 48-bit NMRAnet Node ID to associate alias with
 * @param alias 12-bit alias associated with Node ID
 */
void AliasCache::add(NodeID id, NodeAlias alias)
{
#if 0
    HASSERT(id != 0);
    HASSERT(alias != 0);
    
    Metadata *insert;

    if (freeList)
    {
        /* found an empty slot */
        insert = freeList;
        freeList = insert->next;        
    }
    else
    {
        /* kick out the oldest mapping */
        //map<long long, Metadata*>::iterator oldest = timeMap.begin();
        //insert = oldest->second;
        //aliasMap.erase(insert->alias);
        //idMap.erase(insert->id);
        //timeMap.erase(oldest);
    }
    
    insert->id = id;
    insert->alias = alias;
    insert->timestamp = OSTime::get_monotonic();
    
    //aliasMap[alias] = insert;
    //idMap[id] = insert;
    //timeMap[insert->timestamp] = insert;
    
    return;
#endif
}

/** Remove an alias from an alias cache.
 * @param alias 12-bit alias associated with Node ID
 */
void AliasCache::remove(NodeAlias alias)
{
    HASSERT(alias != 0);

    RBTree<NodeAlias, Metadata*>::Node *node = aliasTree.remove(alias);
    if (node)
    {
        idTree.remove(node->value->id);
        timeTree.remove(node->value->timestamp.value);
        
        node->value->next = freeList;
        freeList = node->value;
    }
    
}

/** Lookup a node's alias based on its Node ID.
 * @param id Node ID to look for
 * @return alias that matches the Node ID, else 0 if not found
 */
NodeAlias AliasCache::lookup(NodeID id)
{
    HASSERT(id != 0);

    RBTree<NodeID, Metadata*>::Node *node = idTree.find(id);
    if (node)
    {
        /* found a match */
        touch(node->value);
        return node->value->alias;
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
    HASSERT(alias != 0);

    RBTree<NodeAlias, Metadata*>::Node *node = aliasTree.find(alias);
    if (node)
    {
        /* found a match */
        touch(node->value);
        return node->value->id;
    }
    
    /* no match found */
    return 0;
}

/** Call the given callback function once for each alias tracked.
 * @param callback method to call
 * @param context context pointer to pass to callback
 */
void AliasCache::for_each(void (*callback)(void*, NodeID, NodeAlias), void *context)
{
    HASSERT(callback != NULL);

    for (RBTree<NodeAlias, Metadata*>::Node *node = aliasTree.first();
         node !=NULL;
         node = aliasTree.next(node))
    {
        (*callback)(context, node->value->id, node->value->alias);
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

};
