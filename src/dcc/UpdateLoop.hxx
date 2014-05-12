/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file UpdateLoop.hxx
 *
 * Interface to the global update loop.
 *
 * @author Balazs Racz
 * @date 10 May 2014
 */

#ifndef _DCC_UPDATELOOP_HXX_
#define _DCC_UPDATELOOP_HXX_

template<class T> class FlowInterface;
template<class T> class Buffer;
namespace dcc { class Packet; }
typedef FlowInterface<Buffer<dcc::Packet> > PacketFlowInterface;

namespace dcc {

class PacketSource;

/** Notifies the update processor that a modification happened to one of the
 * packet sources. Update processors will typically prioritize this packet
 * source in the packet generation loop.
 * @param source is the packet source that experienced a change
 * @param code is a source-specific value that will be sent to the source in
 * the get_next_packet callback. It should not be zero.*/
void packet_processor_notify_update(PacketSource* source, unsigned code);

}  // namespace dcc

#endif //_DCC_UPDATELOOP_HXX_
