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
 * \file TrainStorage.hxx
 *
 * Storage policies for trains in the automatic logon example.
 *
 * @author Balazs Racz
 * @date 14 Aug 2021
 */

#ifndef _APPLICATION_CS_LOGON_TRAINSTORAGE_HXX_
#define _APPLICATION_CS_LOGON_TRAINSTORAGE_HXX_

#include "dcc/Defs.hxx"
#include "dcc/Loco.hxx"
#include "dcc/LogonModule.hxx"
#include "openlcb/EventHandlerTemplates.hxx"
#include "openlcb/TractionTrain.hxx"
#include "utils/Uninitialized.hxx"

class ModuleBase {
public:
    struct Storage
    {
        uninitialized<dcc::Dcc28Train> impl_;
        uninitialized<openlcb::TrainNodeForProxy> node_;
        uninitialized<
            openlcb::FixedEventProducer<openlcb::TractionDefs::IS_TRAIN_EVENT>>
            eventProducer_;
    };
};

class TrainLogonModule : public dcc::ParameterizedLogonModule<ModuleBase> {
public:
    TrainLogonModule(openlcb::TrainService *s)
        : trainService_(s)
    {
    }

    using Base = ParameterizedLogonModule<ModuleBase>;
    
    void assign_complete(unsigned loco_id)
    {
        Base::assign_complete(loco_id);
        auto& t = locos_[loco_id];
        uint8_t part = (t.assignedAddress_ >> 8) & dcc::Defs::ADR_MASK;
        
        if (part == dcc::Defs::ADR_MOBILE_SHORT) {
            t.impl_.emplace(dcc::DccShortAddress(t.assignedAddress_ & 0x7f));
        }
        else if (part >= dcc::Defs::ADR_MOBILE_LONG &&
            part <= dcc::Defs::MAX_MOBILE_LONG)
        {
            t.impl_.emplace(dcc::DccLongAddress(
                t.assignedAddress_ - (dcc::Defs::ADR_MOBILE_LONG << 8)));
        } else {
            // Not a mobile decoder. We don't have an implementation for those
            // yet.
            return;
        }
        t.node_.emplace(trainService_, t.impl_.get_mutable());
        t.eventProducer_.emplace(t.node_.get_mutable());
    }
    
private:
    openlcb::TrainService* trainService_;
};


#endif // _APPLICATION_CS_LOGON_TRAINSTORAGE_HXX_
