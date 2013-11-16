/** \copyright
 * Copyright (c) 2012, Stuart W Baker
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
 * \file nmranet_alias.c
 * This file defines a set of utilities for mapping full 48-bit NMRAnet Node
 * IDs to an alias.
 *
 * @author Stuart W. Baker
 * @date 8 December 2012
 */

#include <limits.h>
#include "os/os.h"
#include "nmranet_types.h"
#include "nmranet_config.h"

#define CONSTANT 0x1B0CA37ABA9 /**< constant for random number generation */

/** Alias cache metadata.
 */
typedef struct alias_cache
{
    node_id_t seed;
    node_id_t *id;
    node_alias_t *alias;
    long long *timestamp;
    size_t size;
} AliasCache;

/** Create a cache of aliases.
 * @param seed 48-bit seed (often a Node ID) used to generate random aliases
 * @param cache_size totoal number of aliases kept by this cache
 * @return handle to the alias cache
 */
alias_cache_t nmranet_alias_cache_create(node_id_t seed, size_t cache_size)
{
    AliasCache *alias_cache = malloc(sizeof(AliasCache));
    
    alias_cache->size = cache_size;
    alias_cache->seed = seed;
    alias_cache->id = malloc(cache_size * sizeof(node_id_t));
    alias_cache->alias = malloc(cache_size * sizeof(node_alias_t));
    memset(alias_cache->id, 0, cache_size * sizeof(node_id_t));
    memset(alias_cache->alias, 0, cache_size * sizeof(node_alias_t));
    alias_cache->timestamp = malloc(cache_size * sizeof(long long));
    
    return alias_cache;
}

/** Add an alias to an alias cache.
 * @param cache alias cache to add alias to
 * @param id 48-bit NMRAnet Node ID to associate alias with
 * @param alias 12-bit alias associated with Node ID
 */
void nmranet_alias_add(alias_cache_t cache, node_id_t id, node_alias_t alias)
{
    AliasCache  *alias_cache = cache;
    long long    time        = LLONG_MAX;
    unsigned int index       = 0;

    if (cache == NULL || id == 0 || alias == 0)
    {
        /* invalid parameter */
        return;
    }

    for (unsigned int i = 0; i < alias_cache->size; i++)
    {
        if (alias_cache->alias[i] == 0)
        {
            /* found an empty slot */
            index = i;
            break;
        }
        else if (alias_cache->timestamp[i] < time)
        {
            /* found a new oldest slost */
            time = alias_cache->timestamp[i];
            index = i;
        }
    }
    
    alias_cache->id[index] = id;
    alias_cache->alias[index] = alias;
    alias_cache->timestamp[index] = os_get_time_monotonic();
    
    return;
}

/** Remove an alias from an alias cache.
 * @param cache alias cache to remove alias from
 * @param alias 12-bit alias associated with Node ID
 */
void nmranet_alias_remove(alias_cache_t cache, node_alias_t alias)
{
    AliasCache  *alias_cache = cache;
    
    if (cache == NULL || alias == 0)
    {
        /* invalid parameter */
        return;
    }

    for (unsigned int i = 0; i < alias_cache->size; i++)
    {
        if (alias == alias_cache->alias[i])
        {
            alias_cache->alias[i] = 0;
            return;
        }
    }
}

/** Lookup a node's alias based on its Node ID.
 * @param cache alias cache to look for an alias in
 * @param id Node ID to look for
 * @return alias that matches the Node ID, else 0 if not found
 */
node_alias_t nmranet_alias_lookup(alias_cache_t cache, node_id_t id)
{
    AliasCache  *alias_cache = cache;
    
    if (cache == NULL || id == 0)
    {
        /* invalid parameter */
        return 0;
    }

    for (unsigned int i = 0; i < alias_cache->size; i++)
    {
        if (alias_cache->id[i] == id)
        {
            /* found a match */
            return alias_cache->alias[i];
        }
    }
    
    /* no match found */
    return 0;
}

/** Lookup a node's ID based on its alias.
 * @param cache alias cache to look for a Node ID in
 * @param alias alias to look for
 * @return Node ID that matches the alias, else 0 if not found
 */
node_id_t nmranet_alias_lookup_id(alias_cache_t cache, node_alias_t alias)
{
    AliasCache  *alias_cache = cache;
    
    if (cache == NULL || alias == 0)
    {
        /* invalid parameter */
        return 0;
    }

    for (unsigned int i = 0; i < alias_cache->size; i++)
    {
        if (alias_cache->alias[i] == alias)
        {
            /* found a match */
            return alias_cache->id[i];
        }
    }
    
    /* no match found */
    return 0;
}

/** Call the given callback function once for each alias tracked.
 * @param cache alias cache to look for a Node ID in
 * @param callback method to call
 * @param context context pointer to pass to callback
 */
void nmranet_alias_for_each(alias_cache_t cache, void (*callback)(void*, node_id_t, node_alias_t), void *context)
{
    AliasCache  *alias_cache = cache;
    
    if (callback == NULL)
    {
        /* invalid parameter */
        return;
    }

    for (unsigned int i = 0; i < alias_cache->size; i++)
    {
        if (alias_cache->alias[i] != 0)
        {
            (*callback)(context, alias_cache->id[i], alias_cache->alias[i]);
        }
    }
}

/** Generate a 12-bit pseudo-random alias for a givin alias cache.
 * @param cache alias cache to generate a unique new alias for
 * @return pseudo-random 12-bit alias, an alias of zero is invalid
 */
node_alias_t nmranet_alias_generate(alias_cache_t cache)
{
    AliasCache  *alias_cache = cache;
    node_alias_t alias;

    #define SEED (alias_cache->seed)

    if (cache == NULL)
    {
        /* invalid parameter */
        return 0;
    }

    do
    {
        /* calculate the alias given the current seed */
        alias = (SEED ^ (SEED >> 12) ^ (SEED >> 24) ^ (SEED >> 36)) & 0xfff;

        /* calculate the next seed */
        SEED = ((((1 << 9) + 1) * (SEED) + CONSTANT)) & 0xffffffffffff;
    } while (alias == 0 || nmranet_alias_lookup_id(cache, alias) != 0);

    /* new random alias */
    return alias;
}

