/** \copyright
 * Copyright (c) 2016, Stuart W Baker
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without written consent.
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
 * \file withrottle/Defs.hxx
 *
 * This file provides basic WiThrottle definitions.
 *
 * @author Stuart Baker
 * @date 17 December 2016
 */

#ifndef _WITHROTTLE_DEFS_HXX_
#define _WITHROTTLE_DEFS_HXX_

#include <string>

#include "openlcb/TractionThrottle.hxx"

namespace withrottle
{

/** Locomotive address */
struct LocoAddress
{
    uint16_t address : 14; /**< 14-bit DCC address */
    uint16_t addressType : 1; /**< true if long address, else short address */
    uint16_t inUse : 1; /**< reserved bit space */
};

/** Roster entry item.
 */
struct RosterEntry
{
    string name; /**< display name of entry */
    LocoAddress address; /**< locomotive adress */
};

/** type of command.
 */
enum CommandType
{
    PRIMARY    = 'T', /**< primary throttle */
    SECONDARY  = 'S', /**< secondary throttle */
    MULTI      = 'M', /**< multi throttle */
    HEX_PACKET = 'D', /**< hex packet for command station */
    HEARTBEAT  = '*', /**< send hardbeat or set heartbeat on/off */
    SET_NAME   = 'N', /**< set the device name */
    SET_ID     = 'H', /**< set the device id */
    PANEL      = 'P', /**< send panel command */
    ROSTER     = 'R', /**< send roster command */
    QUIT       = 'Q', /**< device has quit */
    TYPE_MASK  = 0xFF, /**< exact mask for dispatcher */
};

/** type of multi throttle command.
 */
enum CommandMultiType
{
    ACTION = 'A', /**< pefrom an action */
    ADD    = '+', /**< Add a locomotive to the throttle */
    REMOVE = '-', /**< remove a locomotive from the throttle */
};

/** type of throttle command.
 */
enum CommandSubType
{
    VELOCITY     = 'V', /**< velocity command */
    ESTOP        = 'X', /**< emergency stop */
    FUNCTION     = 'F', /**< function key */
    FORCE        = 'f', /**< force function */
    DIRECTION    = 'R', /**< set direction */
    RELEASE      = 'r', /**< release a loco */
    DISPATCH     = 'd', /**< dispatch a loco */
    ADDR_LONG    = 'L', /**< set long address */
    ADDR_SHORT   = 'S', /**< set short address */
    ADDR_ROSTER  = 'E', /**< set address from roster entry */
    CONSIST      = 'C', /**< consist */
    CONSIST_LEAD = 'c', /**< consist lead from roster entry */
    IDLE         = 'I', /**< idle, set speed to 0 */
    SS_MODE      = 's', /**< set speed step mode */
    MOMENTARY    = 'm', /**< momentary */
    QUERY        = 'q', /**< query about current speed, direction, etc... */
    SUBTYPE_MASK = 0xFF, /**< exact mask for dispatcher */
};

/** Command from the throttle.
 */
class ThrottleCommand
{
public:
    /** type of the dispatcher criteria */
    typedef CommandType id_type;

    CommandType commandType; /**< type of command */
    CommandMultiType commandMultiType; /**< type of multi throttle command */
    CommandSubType commandSubType; /**< type of throttle command */
    string train; /**< The train description */
    string payload; /**< the command payload */

    /** @returns the unique identifier of the reply message */
    id_type id()
    {
        return commandType;
    };
};

/** Server state machine commands.
 */
enum ServerState
{
    STATE_COMMAND = 0, /**< look for command */
    STATE_MULTI_TYPE, /**< look for the multi-throttle type */
    STATE_TRAIN, /**< look for the train */
    STATE_SUBCOMMAND, /**< look for sub-command */
    STATE_PAYLOAD, /**< look for data */
    STATE_NEXT, /**< look for next command */
};

/** The interface definitions for WiThrottle.
 */
struct Defs
{
    static constexpr const int DEFAULT_PORT = 12090;
    /** Protocol version string */
    static constexpr const char *PROTOCOL_VERSION = "VN2.0";

    /** Track power on string */
    static constexpr const char *TRACK_POWER_OFF = "PPA0";

    /** Track power off string */
    static constexpr const char *TRACK_POWER_ON = "PPA1";

    /** Track power unknown string */
    static constexpr const char *TRACK_POWER_UNKNOWN = "PPA2";

    /** Heartbeat timeout string */
    static constexpr const char *HEARTBEAT_TIMEOUT = "*10";

    /** Get the init command string.
     * @return init string
     */
    static string get_init_string()
    {
        string init(PROTOCOL_VERSION);

        init.append("\n\nRL0");
        init.append(2, '\n');
        init.append(Defs::TRACK_POWER_ON);
        init.append(2, '\n');
        init.append("PTT\n\n");
        init.append("PRT\n\n");
        init.append("RCC0\n\n");
        init.append(Defs::HEARTBEAT_TIMEOUT);
        init.append(2, '\n');

        return init;
    }

    /** Get the function status string.
     * @param loco WiThrottle train handle string
     * @param number function number
     * @param state function state
     * @return function status string
     */
    static string get_function_status_string(const char *loco, int number,
                                             bool state)
    {
        string status("MTA");
        status.append(loco);
        status.append("<;>F");
        status.append(1, state ? '1' : '0');
        if (number < 10)
        {
            status.append(1, '0' + number);
        }
        else
        {
            status.append(1, '0' + (number / 10));
            status.append(1, '0' + (number % 10));
        }
        status.append("\n\n");

        return status;
    }

    /** Get the locomotive status command string.
     * @param throttle OpenLCB assigned openLCB throttle
     * @return locomotive status string
     */
    static string get_loco_status_string(openlcb::TractionThrottle *throttle,
                                         const char *loco)
    {
        string status("MT+");
        status.append(loco);
        status.append("<;>\n\n");
        for (int i = 0; i <=28; ++i)
        {
            status.append(get_function_status_string(loco, i,
                                                     throttle->get_fn(i)));
        }
        status.append("MTA");
        status.append(loco);
        status.append("<;>V0\n\nMTA");
        status.append(loco);
        status.append("<;>R1\n\nMTA");
        status.append(loco);
        status.append("<;>S1\n\n");
#if 0
        string status("TL6915\n\nTF00\n\nTF01\n\nTF02\n\nTF03\n\nTF04\n\nTF05\n\n");
        status.append("TF06\n\nTF07\n\nTF08\n\nTF09\n\nTF010\n\nTF011\n\nTF012\n\n");
        status.append("TF013\n\nTF014\n\nTF015\n\nTF016\n\nTF017\n\nTF018\n\n");
        status.append("TF019\n\nTF020\n\nTF021\n\nTF022\n\nTF023\n\nTF024\n\n");
        status.append("TF025\n\nTF026\n\nTF027\n\nTF028\n\nTV0\n\nTR1\n\nTs1\n\n");
#endif
        return status;
    }
};

} /* namespace withrottle */

#endif /* _WITHROTTLE_DEFS_HXX_ */
