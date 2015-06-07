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

#include <set>

#include "executor/Service.hxx"
#include "nmranet/Node.hxx"
#include "nmranet/TractionDefs.hxx"
#include "nmranet/TrainInterface.hxx"

namespace nmranet
{


class TrainService;

/// Virtual node class for an OpenLCB train protocol node.
///
/// Usage:
///   - Create a TrainImpl defining how to send the commands to the hardware.
///   - Create a TrainNode and pass it the pointer to the implementation.
///
/// for train implementations see @ref LoggingTrain, @ref dcc::Dcc28Train, @ref
/// dcc::MMNewTrain etc.
class TrainNode : public Node
{
public:
    TrainNode(TrainService *service, TrainImpl *train);

    NodeID node_id() OVERRIDE;
    If *interface() OVERRIDE;
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

    NodeHandle get_controller()
    {
        return controllerNodeId_;
    }

    void set_controller(NodeHandle id)
    {
        controllerNodeId_ = id;
    }

private:
    unsigned isInitialized_ : 1;

    TrainService *service_;
    TrainImpl *train_;
    /// Controller node that is assigned to run this train. 0 if none.
    NodeHandle controllerNodeId_;
};

/// Collection of control flows necessary for implementing the Traction
/// Protocol.
///
/// usage: instantiate for the given interface. Pass the pointer to the train
/// nodes upon their construction.
class TrainService : public Service, private Atomic
{
public:
    TrainService(If *interface);
    ~TrainService();

    If *interface()
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

    If *interface_;
    /** List of train nodes managed by this Service. */
    std::set<TrainNode *> nodes_;
};

} // namespace nmranet

#endif // _NMRANET_TRACTIONTRAIN_HXX_
