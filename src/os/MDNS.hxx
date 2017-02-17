/** @copyright
 * Copyright (c) 2017, Stuart W Baker
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
 * @file MDNS.hxx
 *
 * A simple abstraction to publish mDNS sevices.
 *
 * @author Stuart Baker
 * @date 30 January 2017
 */

#ifndef _OS_MDNS_HXX_
#define _OS_MDNS_HXX_

#if !defined (__linux__) && \
    !defined (TARGET_IS_CC3200)
#error target not support for MDNS
#endif

#if defined (__linux__)
#include <stdio.h>
#include <semaphore.h>
#include <avahi-client/client.h>
#include <avahi-common/error.h>
#include <avahi-common/malloc.h>
#include <avahi-client/publish.h>
#include <avahi-common/simple-watch.h>
#include "os/OS.hxx"
#endif

#if defined (TARGET_IS_CC3200)
#include <string>
#include "netapp.h"
#undef OK // this is to fix the namespace polution in netapp.h
#endif

#include "utils/macros.h"

/** MDNS abstraction object.
 */
class MDNS
#if defined (__linux__)
    : public OSThread
#endif
{
public:
    /** Constructor.
     */
    MDNS()
#if defined (__linux__)
        : 
          OSThread()
        , sem()
        , group(nullptr)
        , simplePoll(nullptr)
        , client(nullptr)
#endif
    {
#if defined (__linux__)
        start("mDNS Server", 0, 2048);
        printf("client start\n");        
        sem.wait();
#endif
    }

    /** Destructor.
     */
    ~MDNS()
    {
    }

    /** Publish an mDNS name.
     */
    void publish(const char *name, const char *service, uint16_t port)
    {
#if defined (__linux__)
        name = avahi_strdup(name);

        if (!group)
        {
            group = avahi_entry_group_new(client, entry_group_callback, this);
            HASSERT(group);
        }

        int result = avahi_entry_group_add_service(group, AVAHI_IF_UNSPEC,
                                                   AVAHI_PROTO_UNSPEC,
                                                   (AvahiPublishFlags)0, name,
                                                   service, NULL, NULL,
                                                   port, NULL);

        HASSERT(result == 0);
#elif defined (TARGET_IS_CC3200)
        string full_name(name);
        full_name.append(1, '.');
        full_name.append(service);
        full_name.append(".local");
        sl_NetAppMDNSRegisterService((const signed char*)full_name.c_str(),
                                     full_name.length(),
                                     (const signed char*)"OLCB", strlen("OLCB"), 
                                     port, 200, 0);
#endif
    }

    void commit()
    {
#if defined (__linux__)
        avahi_entry_group_commit(group);
#endif
    }

private:
#if defined (__linux__)
    /** Avahi group callback.
     */
    static void entry_group_callback(AvahiEntryGroup *g,
                                     AvahiEntryGroupState state, void *userdata)
    {
        MDNS *mdns = static_cast<MDNS *>(userdata);
        mdns->group = g;
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
     * @return should never return
     */
    void *entry() override
    {
        int error;

        simplePoll = avahi_simple_poll_new();
        HASSERT(simplePoll);

        client = avahi_client_new(avahi_simple_poll_get(simplePoll),
                                  (AvahiClientFlags)0, client_callback, this,
                                  &error);
        printf("client created\n");
        HASSERT(client);

        sem.post();
        avahi_simple_poll_loop(simplePoll);

        printf("mdns_thread exit\n");

        return nullptr;
    }

    /** Synchronize the startup of AVAHI */
    OSSem sem;

    /** AVAHI group */
    AvahiEntryGroup *group;

    /** AVAHI polling client */
    AvahiSimplePoll *simplePoll;

    /** AVAHI client */
    AvahiClient *client;
#endif

    DISALLOW_COPY_AND_ASSIGN(MDNS);
};

#endif /* _OS_MDNS_HXX_ */
