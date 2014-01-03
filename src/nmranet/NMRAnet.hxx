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
 * \file NMRAnet.hxx
 * Container for the very few global NMRAnet objects
 *
 * @author Stuart W. Baker
 * @date 26 September 2013
 */

#ifndef _NMRAnet_hxx_
#define _NMRAnet_hxx_

#include "executor/Executor.hxx"

namespace NMRAnet
{

/** The single executor instance used for all NMRAnet platform logic. */
extern Executor<4> *nmranetExecutor;

/** Base identifier for all NMRAnet messages */
#define NMRANET_ID_BASE (1000)

/** Base identifier for all NMRAnet::If messages */
#define NMRANET_IF_BASE (NMRANET_ID_BASE + 0)

/** Base identifier for all NMRAnet::IfCan messages */
#define NMRANET_IF_CAN_BASE (NMRANET_ID_BASE + 20)

/** Base identifier for all NMRAnet::Event messages */
#define NMRANET_EVENT_BASE (NMRANET_ID_BASE + 100)

/** Base identifier for all NMRAnet::Datagram messages */
#define NMRANET_DATAGRAM_BASE (NMRANET_ID_BASE + 200)

/** Base identifier for all NMRAnet::Stream messages */
#define NMRANET_STREAM_BASE (NMRANET_ID_BASE + 300)

}

#endif /* _NMRAnet_hxx_ */
