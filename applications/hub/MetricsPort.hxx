/** \copyright
 * Copyright (c) 2024, Stuart Baker
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
 * \file MetricsPort.hxx
 *
 * Hub port that tracks packet metrics for monitoring.
 *
 * @author Stuart Baker
 * @date 12 August 2024
 */

#ifndef _APPLICATIONS_HUB_METRICSPORT_HXX_
#define _APPLICATIONS_HUB_METRICSPORT_HXX_

#include "executor/StateFlow.hxx"
#include "utils/Hub.hxx"
#include "HubMetrics.hxx"

/// Port that attaches to a CanHubFlow and tracks packet statistics.
/// This port is promiscuous and does not consume packets, it only observes them.
class MetricsPort : public StateFlow<Buffer<CanHubData>, QList<1>>
{
public:
    /// Constructor.
    /// @param service The service to run the state flow on
    /// @param metrics Pointer to the HubMetrics object to update
    MetricsPort(Service *service, HubMetrics *metrics)
        : StateFlow<Buffer<CanHubData>, QList<1>>(service)
        , metrics_(metrics)
    {
    }

    /// Destructor.
    ~MetricsPort()
    {
    }
    
    /// @return the port interface for registering with the hub
    FlowInterface<Buffer<CanHubData>> *get_port()
    {
        return this;
    }

private:
    /// State flow entry point
    Action entry() override
    {
        // Count the packet
        metrics_->record_packet_rx();
        
        // Count the bytes (CAN frame data length)
        auto *data = message()->data();
        if (data)
        {
            // GridConnect format is approximately 25-30 bytes per CAN frame
            // Format: :XhhhhhhhhNdddddddddddddddd;
            // where h = 8 hex digits for ID, d = up to 16 hex digits for data
            // We'll estimate based on actual data length
            const struct can_frame *frame = data;
            size_t gc_bytes = 12 + (frame->can_dlc * 2); // :X...N...;
            metrics_->record_packet_rx(gc_bytes);
        }
        
        // Release the buffer since we're just observing
        return release_and_exit();
    }

    /// Pointer to the metrics object
    HubMetrics *metrics_;
};

#endif // _APPLICATIONS_HUB_METRICSPORT_HXX_
