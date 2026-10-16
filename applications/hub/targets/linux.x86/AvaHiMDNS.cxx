/** \copyright
 * Copyright (c) 2016, Stuart W. Baker
 * All rights reserved
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
 * \file AvaHiMDNS.cxx
 *
 * A simple abstraction to publish mDNS services using Avahi.
 *
 * @author Stuart Baker
 * @date 30 July 2016
 */

#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <avahi-client/client.h>
#include <avahi-client/publish.h>
#include <avahi-common/simple-watch.h>
#include <avahi-common/malloc.h>
#include <avahi-common/error.h>

#include "utils/macros.h"
#include "openlcb/TcpDefs.hxx"

static AvahiEntryGroup *group = nullptr;
static AvahiSimplePoll *simplePoll = nullptr;
static AvahiClient *client = nullptr;
static sem_t wait;

/** Avahi group callback.
 */
static void entry_group_callback(AvahiEntryGroup *g, AvahiEntryGroupState state,
                                 void *userdata)
{
    group = g;
}

/** Avahi client callback.
 */
static void client_callback(AvahiClient *c, AvahiClientState state,
                            void * userdata)
{
    switch (state)
    {
        default:
            break;
        case AVAHI_CLIENT_S_RUNNING:
            printf("client running\n");
            break;
    }
}

/** mdns polling thread.
 * @param unused unused parameter
 * @return should never return
 */
static void *mdns_thread(void *unused)
{
    int error;

    simplePoll = avahi_simple_poll_new();
    HASSERT(simplePoll);

    client = avahi_client_new(avahi_simple_poll_get(simplePoll),
                              (AvahiClientFlags)0, client_callback, nullptr,
                              &error);
    printf("client created\n");
    HASSERT(client);

    sem_post(&wait);
    avahi_simple_poll_loop(simplePoll);

    return nullptr;
}

/** Start the mDNS client.
 * @brief Initializes and starts the Avahi mDNS client in a separate thread.
 * 
 * This function creates a new thread that runs the Avahi event loop. The calling
 * thread will block until the Avahi client is successfully initialized and running.
 * The created thread is detached and will run indefinitely to handle mDNS operations.
 */
void mdns_client_start()
{
    int result = sem_init(&wait, 0, 0);
    if (result != 0)
    {
        fprintf(stderr, "sem_init failed: %d\n", result);
        return;
    }

    pthread_t thread;
    result = pthread_create(&thread, nullptr, mdns_thread, nullptr);
    if (result != 0)
    {
        fprintf(stderr, "pthread_create failed: %d\n", result);
        sem_destroy(&wait);
        return;
    }
    pthread_detach(thread);
    printf("client start\n");
    
    result = sem_wait(&wait);
    if (result != 0)
    {
        fprintf(stderr, "sem_wait failed: %d\n", result);
    }
}

/** Publish an mDNS name.
 * @brief Publishes a GridConnect CAN TCP service via mDNS using Avahi.
 * @param name The human-readable service name to publish (e.g., "openmrn_hub").
 *             This string will be duplicated internally.
 * @param port The TCP port number on which the service is listening.
 * 
 * The service is published with the type defined in
 * openlcb::TcpDefs::MDNS_SERVICE_NAME_GRIDCONNECT_CAN_TCP.
 * If an entry group does not yet exist, it will be created automatically.
 */
void mdns_publish(const char *name, uint16_t port)
{
    const char *service_name = avahi_strdup(name);

    if (!group)
    {
        group = avahi_entry_group_new(client, entry_group_callback, nullptr);
        if (!group) {
            fprintf(stderr, "avahi_entry_group_new() failed: %s\n",
                avahi_strerror(avahi_client_errno(client)));
        }
        HASSERT(group);
    }

    int result = avahi_entry_group_add_service(group, AVAHI_IF_UNSPEC,
        AVAHI_PROTO_UNSPEC, (AvahiPublishFlags)0, service_name,
        openlcb::TcpDefs::MDNS_SERVICE_NAME_GRIDCONNECT_CAN_TCP, NULL, NULL, port,
        "platform=linux-x86-openmrn", NULL);

    if (result != 0)
    {
        fprintf(stderr, "Error exporting mDNS name (%d) %s\n", result,
            avahi_strerror(result));
    }

    HASSERT(result == 0);
    avahi_entry_group_commit(group);
}
