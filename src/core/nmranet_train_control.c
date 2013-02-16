/** \copyright
 * Copyright (c) 2012, Stuart W Baker
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file nmranet_train_control.c
 * This file handles the NMRAnet datagram protocol for train control.
 *
 * @author Stuart W. Baker
 * @date 11 November 2012
 */

#if 0
#include <stdlib.h>
#include <stdint.h>
#include "core/nmranet_datagram.h"
#include "core/nmranet_train_control.h"

/** All the callbacks for the train control protocol */
typedef struct tc_callbacks
{
    TCEstopCallback tcEstopCallback; /**< emergency stop callback */
    TCVelocityCallback tcVelocityCallback; /**< velocity callback */
    TCVelocityInquireCallback tcVelocityInquireCallback; /**< velocity inquire callback */
    TCVelocityReportCallback tcVelocityReportCallback; /**< velocity report callback */
} TCCallbacks;

/** callback functions for the train control protocol */
static TCCallbacks callbacks;

/** Types of train control commands */
enum command_id
{
    CMD_ESTOP            = 0x00, /**< emergency stop */
    CMD_VELOCITY         = 0x01, /**< command velocity */
    CMD_VELOCITY_INQUIRE = 0x02, /**< inquire about present velocity */
    CMD_VELOCITY_REPORT  = 0x03 /**< report present velocity */
};

#if 0
/** Structure of @ref CMD_VELOCITY payload */
typedef struct velocity
{
    float velocity; /**< tc_velocity in meters per second */
};

/** Structure of @ref CMD_VELOCITY_INQUIRE payload */
typedef struct velocity_inquire
{
    int kind; /**< @ref tc_velocity_kind of speed */
};

/** Structure of @ref CMD_VELOCITY_REPORT payload */
typedef struct velocity_report
{
    float velocity; /**< velocity in meters per second */
    int kind; /**< @ref tc_velocity_kind of speed */
};
#endif

/** Process the e-stop datagram.
 * @param node node handle the datagram was received from
 * @param datagram datagram to process
 */
static void process_estop(node_t node, Datagram *datagram)
{
    if (callbacks.tcEstopCallback)
    {
        (*callbacks.tcEstopCallback)(node, datagram->from);
    }
}

/** Process the velocity datagram.
 * @param node node handle the datagram was received from
 * @param datagram datagram to process
 */
static void process_velocity(node_t node, Datagram *datagram)
{
    if (callbacks.tcEstopCallback)
    {
        (*callbacks.tcEstopCallback)(node, datagram->from);
    }
}

/** Process the datagram automatically for train control.
 * @param node node handle the datagram was received from
 * @param datagram datagram to process
 */
void nmranet_train_control_process(node_t node, Datagram *datagram)
{
    uint8_t command_id = ((uint8_t*)(datagram->data))[1];

    switch (command_id)
    {
        default:
            break;
        case CMD_ESTOP:
            process_estop(node, datagram);
            break;
        case CMD_VELOCITY:
            process_velocity(node, datagram);
            break;
        case CMD_VELOCITY_INQUIRE:
            break;
        case CMD_VELOCITY_REPORT:
            break;
    }
}

/** Register application callbacks for the train control protocol.
 * @param tcEstopCallback emergency stop callback
 * @param tcVelocityCallback velocity callback
 * @param tcVelocityInquireCallback velocity inquire callback
 * @param tcVelocityReportCallback velocity report callback
 */
void nmranet_train_control_callbacks(TCEstopCallback tcEstopCallback,
                                     TCVelocityCallback tcVelocityCallback,
                                     TCVelocityInquireCallback tcVelocityInquireCallback,
                                     TCVelocityReportCallback tcVelocityReportCallback)
{
    callbacks.tcEstopCallback = tcEstopCallback;
    callbacks.tcVelocityCallback = tcVelocityCallback;
    callbacks.tcVelocityInquireCallback = tcVelocityInquireCallback;
    callbacks.tcVelocityReportCallback = tcVelocityReportCallback;
}
#endif

