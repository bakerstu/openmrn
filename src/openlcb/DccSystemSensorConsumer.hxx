/** \copyright
 * Copyright (c) 2026, Balazs Racz
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
 * \file DccSystemSensorConsumer.hxx
 *
 * Consumer class for DCC System Sensor events.
 *
 * @author Balazs Racz
 * @date 18 Jan 2026
 */

#ifndef _OPENLCB_DCCSYSTEMSENSORCONSUMER_HXX_
#define _OPENLCB_DCCSYSTEMSENSORCONSUMER_HXX_

#include "openlcb/TractionDefs.hxx"
#include "openlcb/WellKnownEventRangeConsumer.hxx"

namespace openlcb
{

/// Well-Known event consumer for DCC System Sensors. The state is stored in
/// memory and no output is generated.
class DccSystemSensorConsumer : public WellKnownEventRangeConsumer
{
public:
    static const EventRangeConfig *get_config()
    {
        static constexpr EventRangeConfig cfg = {
            TractionDefs::ACTIVATE_DCC_SYSTEM_SENSOR_EVENT_BASE,
            TractionDefs::INACTIVATE_DCC_SYSTEM_SENSOR_EVENT_BASE,
            12,  // 4096 events (12 bits)
            4096 // 4096 state bits (4096 sensors)
        };
        return &cfg;
    }

    /// Constructs a listener for DCC system sensor events.
    /// @param node is the virtual node that will be listening for events.
    DccSystemSensorConsumer(Node *node)
        : WellKnownEventRangeConsumer(node, get_config())
    {
    }

    /// Destructor.
    ~DccSystemSensorConsumer()
    {
    }

    /// Checks if a sensor is currently active (on/high).
    /// @param sensor_num the binary sensor address (0-4095).
    /// @return true if the sensor is active, false otherwise.
    bool is_sensor_active(uint32_t sensor_num) const
    {
        return get_state(sensor_num);
    }

    /// Checks if the state of a sensor is known.
    /// @param sensor_num the binary sensor address (0-4095).
    /// @return true if the sensor state is known, false otherwise.
    bool is_sensor_known(uint32_t sensor_num) const
    {
        return is_state_known(sensor_num);
    }

protected:
    /// Parses an event into identifying properties.
    bool parse_event(EventId event, uint32_t *address, bool *value) override
    {
        if (event >= cfg_->activate_base &&
            event < cfg_->activate_base + (1UL << cfg_->mask_bits))
        {
            *value = true;
            *address = event - cfg_->activate_base;
        }
        else if (event >= cfg_->inactivate_base &&
            event < cfg_->inactivate_base + (1UL << cfg_->mask_bits))
        {
            *value = false;
            *address = event - cfg_->inactivate_base;
        }
        else
        {
            return false;
        }
        return true;
    }

    /// Perform action after state change.
    void action_impl() override
    {
        // No action required for system sensors, they just update internal
        // state.
    }
};

} // namespace openlcb

#endif // _OPENLCB_DCCSYSTEMSENSORCONSUMER_HXX_
