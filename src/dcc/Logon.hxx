/** \copyright
 * Copyright (c) 2021, Balazs Racz
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
 * \file Logon.hxx
 * Control flows for supporting DCC Automatic Logon.
 *
 * @author Balazs Racz
 * @date 12 Aug 2021
 */

#ifndef _DCC_LOGON_HXX_
#define _DCC_LOGON_HXX_

#include "dcc/PacketSource.hxx"
#include "dcc/TrackIf.hxx"
#include "dcc/UpdateLoop.hxx"
#include "executor/StateFlow.hxx"

namespace dcc
{

/// This class needs to be a base class for the template argument of the Logon
/// Handler.
class LogonHandlerModule
{

}; // LogonHandlerModule

/// Handles the automatic logon flow for DCC decoders.
template <class Module>
class LogonHandler : public NonTrainPacketSource, public StateFlowBase, public RailcomHubPortInterface
{
public:
    /// Constructor
    ///
    /// @param service points to the executor to use.
    /// @param track pointer to the track interface to send DCC packets to.
    /// @param rcom_hub will register to this railcom hub to get feedback.
    LogonHandler(Service *service, TrackIf *track, RailcomHubFlow* rcom_hub)
        : StateFlowBase(service)
        , trackIf_(track)
    {
    }

    /// Initiates a logon sequence at startup.
    void startup_logon()
    {
        start_flow(STATE(startup_logon));
    }
private:
    Action startup_logon() {
        
    }

    
    /// If we need to send packets to the track, we can do it here directly.
    TrackIf *trackIf_;
}; // LogonHandler

} // namespace dcc

#endif // _DCC_LOGON_HXX_
