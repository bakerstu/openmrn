/** \copyright
 * Copyright (c) 2016, Sidney McHarg
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
 * \file FreeRTOSTCP.cxx
 * This file provides the interface to the FreeRTOSPlus TCP stack.
 * Based on work by Stuart W. Baker
 *
 * @author Sidney McHarg
 * @date 21 March 2016
 */

#ifndef _FREERTOS_DRIVERS_NET_FREERTOSTCP_FREERTOSTCP_HXX_
#define _FREERTOS_DRIVERS_NET_FREERTOSTCP_FREERTOSTCP_HXX_

#include "os/OS.hxx"
#include "utils/Singleton.hxx"

class FreeRTOSTCPSocket;

/** local declaraction of Socket_t to avoid dependency on FreeRTOSTCP headers */
typedef void *Socket_t;

/** Provides the startup and mantainance methods for configuring and using the
 * FreeRTOSTCP stack.  This is designed to be a singleton.  It should only
 * be instantiated once.
 */
class FreeRTOSTCP : public Singleton<FreeRTOSTCP>
{
public:
    /** Constructor.
     */
    FreeRTOSTCP();

    /** Destructor.
     */
    ~FreeRTOSTCP()
    {
    }

    /** Startup the networking processes.
     */
    void start();

private:
    /** Thread that will manage the network connection.
     * @param context context passed into the stack.
     */
    static void *net_task_entry(void *context)
    {
        instance()->net_task();
        return nullptr;
    }

    /** Thread that will manage the net connection inside object context.
     */
    void net_task();

    /** Asynchronously wakeup the select call.
     * @param data -1 for no action, else socket descriptor if socket shall be
     *             closed.
     */
    void select_wakeup(Socket_t data = nullptr);

    /** Add socket to the read fd set.
     * @param socket socket descriptor to add
     */
    void fd_set_read(Socket_t socket);

    /** Add socket to the write fd set.
     * @param socket socket descriptor to add
     */
    void fd_set_write(Socket_t socket);

    static FreeRTOSTCP *instance_; /**< singleton instance pointer. */
    uint32_t ipAddress;            /**< assigned IP adress */

    Socket_t wakeup; /**< signal socket to wakeup select() */

    /** allow access to private members from CC32xxSocket */
    friend class FreeRTOSTCPSocket;

    DISALLOW_COPY_AND_ASSIGN(FreeRTOSTCP);
};

#endif /* _FREERTOS_DRIVERS_NET_FREERTOSTCP_FREERTOSTCP_HXX_ */
