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
 * \file nmranet_if.c
 * This file defines a generic NMRAnet interface.
 *
 * @author Stuart W. Baker
 * @date 13 August 2012
 */

#if 0
#include <sys/tree.h>
#include "if/nmranet_if.h"
#include "core/nmranet_buf.h"
#include "os/os.h"

/** Mutual exclusion for interfaces. */
static os_mutex_t mutex = OS_MUTEX_INITIALIZER;

/** One time initialization for the NMRAnet interfaces.
 */
static void if_init(void)
{
}

/** Initialize the network stack.
 * @param node_id Node ID used to identify the built in bridge, 0 for no bridge
 */
void nmranet_init(node_id_t node_id)
{
    bridgeId = node_id;
}

/** Initialize a NMRAnet interface.
 * @param nmranet_if instance of this interface
 */
void nmranet_if_init(NMRAnetIF *nmranet_if)
{
    os_mutex_lock(&mutex);
    os_thread_once(&if_once, if_init);
    if (nmranet_if != NULL)
    {
        /* add our newly created interface to our list of interfaces */
        NMRAnetIF * current = head;
        while (current->next != NULL)
        {
            current = current->next;
        }
        current->next = nmranet_if;
        nmranet_if->next = NULL;
        /* identify everyone on this segment */
        if (bridgeId)
        {
            (*nmranet_if->write)(nmranet_if, MTI_VERIFY_NODE_ID_GLOBAL, bridgeId, 0, NULL);
        }
    }
    os_mutex_unlock(&mutex);
}
#endif
