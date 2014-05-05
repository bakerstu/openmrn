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

namespace NMRAnet
{

typedef float SpeedType;

class TrainImpl
{
public:
    /** Sets the speed of the locomotive.
     * @param speed is the requested scale speed in m/s. The sign of the number
     * means the direction.
     */
    virtual void set_speed(SpeedType speed) = 0;
    /** Returns the last set speed of the locomotive. */
    virtual SpeedType get_speed() = 0;

    /** Sets the value of a function.
     * @param address is a 24-bit address of the function to set. For legacy DCC
     * locomotives, see @ref TractionDefs for the address definitions (0=light,
     * 1-28= traditional function buttons).
     * @param value is the function value. For binary functions, any non-zero
     * value sets the function to on, zero sets it to off.*/
    virtual void set_fn(unsigned address, uint16_t value) = 0;

    /** @returns the value of a function. */
    virtual uint16_t get_fn(unsigned address) = 0;

    /** @returns the legacy (DCC) address of this train. This value is used in
     * determining the train's NMRAnet NodeID.
     * @TODO(balazs.racz) This function should not be here. Specifying the
     * NodeID should be more generic, but it is not clear what would be the
     * best interface for that.
     */
    virtual uint32_t legacy_address() = 0;
};

class TrainNode : public AsyncNode
{
public:
    TrainNode(TrainService *service, TrainImpl *train);

    NodeID node_id() OVERRIDE;
    AsyncIf *interface() OVERRIDE
    {
        return service_->interface();
    }
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
    Impl* impl_;

    AsyncIf *interface_;
    /** List of train nodes managed by this Service. */
    std::set<TrainNode> nodes_;
};

} // namespace NMRAnet

#endif // _NMRANET_TRACTIONTRAIN_HXX_
