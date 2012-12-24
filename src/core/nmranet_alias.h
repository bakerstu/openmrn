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
 * \file nmranet_alias.h
 * This file defines a set of utilities for mapping full 48-bit NMRAnet Node
 * IDs to an alias.
 *
 * @author Stuart W. Baker
 * @date 8 December 2012
 */

#ifndef _nmranet_alias_h_
#define _nmranet_alias_h_

#include "nmranet_types.h"

/** Create a cache of aliases.
 * @param seed 48-bit seed (often a Node ID) used to generate random aliases
 * @param cache_size totoal number of aliases kept by this cache
 * @return handle to the alias cache
 */
alias_cache_t nmranet_alias_cache_create(node_id_t seed, size_t cache_size);

/** Add an alias to an alias cache.
 * @param cache alias cache to add alias to
 * @param id 48-bit NMRAnet Node ID to associate alias with
 * @param alias 12-bit alias associated with Node ID
 */
void nmranet_alias_add(alias_cache_t cache, node_id_t id, node_alias_t alias);

/** Remove an alias from an alias cache.
 * @param cache alias cache to remove alias from
 * @param alias 12-bit alias associated with Node ID
 */
void nmranet_alias_remove(alias_cache_t cache, node_alias_t alias);

/** Lookup a node's alias based on its Node ID.
 * @param cache alias cache to look for an alias in
 * @param id Node ID to look for
 * @return alias that matches the Node ID, else 0 if not found
 */
node_alias_t nmranet_alias_lookup(alias_cache_t cache, node_id_t id);

/** Lookup a node's alias based on its Node ID.
 * @param cache alias cache to look for a Node ID in
 * @param alias alias to look for
 * @return Node ID that matches the alias, else 0 if not found
 */
node_id_t nmranet_alias_lookup_id(alias_cache_t cache, node_alias_t alias);

/** Generate a 12-bit pseudo-random alias for a givin alias cache.
 * @param cache alias cache to generate a unique new alias for
 * @return pseudo-random 12-bit alias, an alias of zero is invalid
 */
node_alias_t nmranet_alias_generate(alias_cache_t cache);

#endif /* _nmranet_alias_h_ */
