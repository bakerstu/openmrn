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
 * \file TractionTrain.hxx
 *
 * Defines an NMRAnet Train node.
 *
 * @author Balazs Racz
 * @date 5 May 2014
 */

#ifndef _NMRANET_TRACTIONTRAIN_HXX_
#define _NMRANET_TRACTIONTRAIN_HXX_

#include "nmranet/NMRAnetAsyncNode.hxx"

#include <set>
#include "nmranet/TractionDefs.hxx"

namespace NMRAnet
{


class TrainService;

class TrainNode : public AsyncNode
{
public:
    TrainNode(TrainService *service, TrainImpl *train);

    NodeID node_id() OVERRIDE;
    AsyncIf *interface() OVERRIDE;
    bool is_initialized() OVERRIDE
    {
        return isInitialized_;
    }
    void set_initialized() OVERRIDE
    {
        isInitialized_ = 1;
    }

    TrainImpl *train()
    {
        return train_;
    }

private:
    unsigned isInitialized_ : 1;

    TrainService *service_;
    TrainImpl *train_;
};

class TrainService : public Service, private Atomic
{
public:
    TrainService(AsyncIf *interface);
    ~TrainService();

    AsyncIf *interface()
    {
        return interface_;
    }

    /** Registers a new train with the train service. Will initiate a node
        initialization flow for the train. */
    void register_train(TrainNode *node);

private:
    struct Impl;
    /** Implementation flows. */
    Impl *impl_;

    AsyncIf *interface_;
    /** List of train nodes managed by this Service. */
    std::set<TrainNode *> nodes_;
};

} // namespace NMRAnet

#endif // _NMRANET_TRACTIONTRAIN_HXX_
