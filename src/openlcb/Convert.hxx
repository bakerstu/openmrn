/** \copyright
 * Copyright (c) 2021, Balazs Racz
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
 * \file Convert.hxx
 *
 * Conversion routines for OpenLCB types.
 *
 * @author Balazs Racz
 * @date 17 Aug 2021
 */

#ifndef _OPENLCB_CONVERT_HXX_
#define _OPENLCB_CONVERT_HXX_

#include <endian.h>
#include <string.h>

#include "openlcb/Defs.hxx"

namespace openlcb
{

/** Convenience function to render a 48-bit NMRAnet node ID into a new buffer.
 *
 * @param id is the 48-bit ID to render.
 * @returns a new buffer (from the main pool) with 6 bytes of used space, a
 * big-endian representation of the node ID.
 */
extern string node_id_to_buffer(NodeID id);
/** Convenience function to render a 48-bit NMRAnet node ID into an existing
 * buffer.
 *
 * @param id is the 48-bit ID to render.
 * @param data is the memory space to write the rendered ID into. There must be
 * at least 6 bytes available at this address.
 */
extern void node_id_to_data(NodeID id, void *data);

/** Converts a 6-byte-long buffer to a node ID.
 *
 * @param buf is a buffer that has to have exactly 6 bytes used, filled with a
 * big-endian node id.
 * @returns the node id (in host endian).
 */
extern NodeID buffer_to_node_id(const string &buf);
/** Converts 6 bytes of big-endian data to a node ID.
 *
 * @param d is a pointer to at least 6 valid bytes.
 * @returns the node ID represented by the first 6 bytes of d.
 */
extern NodeID data_to_node_id(const void *d);

/** Converts an Event ID to a Payload suitable to be sent as an event report. */
extern Payload eventid_to_buffer(uint64_t eventid);

/** Takes 8 bytes (big-endian) from *data, and returns the event id they
 * represent. */
inline uint64_t data_to_eventid(const void *data)
{
    uint64_t ret = 0;
    memcpy(&ret, data, 8);
    return be64toh(ret);
}

/** Formats a payload for response of error response messages such as OPtioanl
 * Interaction Rejected or Terminate Due To Error. */
extern string error_to_buffer(uint16_t error_code, uint16_t mti);

/** Formats a payload for response of error response messages such as Datagram
 * Rejected. */
extern string error_to_buffer(uint16_t error_code);

/** Writes an error code into a payload object at a given pointer. */
extern void error_to_data(uint16_t error_code, void *data);

/** Parses an error code from a payload object at a given pointer. */
extern uint16_t data_to_error(const void *data);

/** Appends an error to the end of an existing buffer. */
extern void append_error_to_buffer(uint16_t error_code, Payload *p);

/** Parses the payload of an Optional Interaction Rejected or Terminate Due To
 * Error message.
 * @param payload is the contents of the incoming addressed message.
 * @param error_code will hold the 2-byte error code, or ERROR_PERMANENT if not
 * specified
 * @param mti will hold the MTI value, or 0 if not specified
 * @param error_message will hold all remaining bytes that came with the error
 * message.
 */
extern void buffer_to_error(const Payload &payload, uint16_t *error_code,
    uint16_t *mti, string *error_message);

/// Generates the payload for an OIR or TDE message.
/// @param error_code the 16-bit ErrorCodes value.
/// @param incoming_mti the MTI of the message that this error should refer
/// to.
extern Payload error_payload(uint16_t error_code, Defs::MTI incoming_mti);

/** A global class / variable for empty or not-yet-initialized payloads. */
extern string EMPTY_PAYLOAD;

/// @return the high 4 bytes of a node ID. @param id is the node ID.
inline unsigned node_high(NodeID id)
{
    return id >> 32;
}
/// @return the low 4 bytes of a node ID. @param id is the node ID.
inline unsigned node_low(NodeID id)
{
    return id & 0xffffffffU;
}

} // namespace openlcb

#endif
