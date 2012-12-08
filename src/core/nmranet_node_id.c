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
 * \file nmranet_node_id.c
 * This file defines NMRAnet node id and alias handling.
 *
 * @author Stuart W. Baker
 * @date 23 November 2012
 */

#include <stdlib.h>
#include "if/nmranet_if.h"
#include "nmranet_types.h"

#if 0
/** Metadata for a node alias to network interface pair */
typedef struct id_alias
{
    NMRAnetIF interface; /**< interface this alias belongs to */
    uint16_t alias; /**< alias on this interface */
    struct id_alias *next; /**< next link in the list */
};

/** Metadata for a given node ID */
typedef struct node_id
{
    uint64_t id; /**< 48-bit node id for this node */
    struct id_alias *head; /**< list of aliases for this node */
    unsigned int count; /**< number of references to this node id */
} NodeId;

/** Red Black tree node for sorting by alias.
 */
struct alias_node
{
    /** entry metadata */
    RB_ENTRY(alias_node) entry;
    union
    {
        node_alias_t alias;
        int16_t key;
    };
    NodeId *nodeId; /**< the metadata for the node id */
};

/** Red Black tree node for sorting by node ID.
 */
struct id_node
{
    /** entry metadata */
    RB_ENTRY(id_node) entry;
    union
    {
        node_id_t id;
        int64_t key;
    };
    NodeId *nodeId; /**< the metadata for the node id */
};
    
/** Mathematically compare two aliases
 * @param a first node to compare
 * @param b second node to compare
 * @return (a - b)
 */
static int16_t alias_compare(struct alias_node *a, struct alias_node *b)
{
    return a->key - b->key;
}

/** Mathematically compare two Node ID's.
 * @param a first node to compare
 * @param b second node to compare
 * @return (a - b)
 */
static int64_t id_compare(struct id_node *a, struct id_node *b)
{
    return a->key - b->key;
}

/** The alias tree type. */
RB_HEAD(alias_tree, alias_node);
/** The alias tree methods. */
RB_GENERATE_STATIC(alias_tree, alias_node, entry, alias_compare);
/** The id tree type. */
RB_HEAD(id_tree, id_node);
/** The id tree methods. */
RB_GENERATE_STATIC(id_tree, id_node, entry, id_compare);
/** The id tree head. */
static struct id_tree idHead = RB_INITIALIZER(&idHead);

/** Mutual exclusion for nodes */
static os_mutex_t mutex = OS_MUTEX_INITIALIZER;

/** alias interfaces */
typedef struct alias_if_tree
{
    NMRAnetIF *nmranet_if; /**< interface for this alias tree */
    struct alias_tree aliasHead; /**< The alias tree head. */
} AliasIFTree;

node_id_t node_id_create(uint64_t id)
{
    struct id_node *id_node;
    struct id_node  id_lookup;
    node_id_t node_id;
    
    if (id == 0)
    {
        return NULL;
    }

    id_lookup.id = id;
    os_mutex_lock(&mutex);
    id_node = RB_FIND(id_tree, &idHead, *id_lookup)
    if (id_node)
    {
        id_node->nodeId->count++;
    }
    else
    {
        id_node = malloc(sizeof(struct id_node));
        id_node->nodeId = malloc(sizeof(NodeId));
        id_node->nodeId->id = id;
        id_node->nodeId->head = NULL;
        id_node->nodeId->count = 1;
        RB_INSERT(id_tree, &idHead, id_node);
    }
    node_id = id_node->nodeId;
    os_mutex_unlock(&mutex);

    return node_id;
}

node_id_t node_id_create_alias(NMRAnetIF *nmranet_if, uint16_t alias)
{
    
}

int node_id_compare(node_id_t a, node_id_t b)
{
    NodeId *id_a = a;
    NodeId *id_b = b;
    
    if (id_a->id == id_b->id)
    {
        return 0;
    }

    return 1;
}

/** Get the 48-bit node id for this node
 * @param node_id node_id to get the full id of
 * @param full 48-bit node id
 */
uint64_t node_id_value(node_id_t node_id)
{
    NodeId *id = node_id

    return id->id;
}
#endif


