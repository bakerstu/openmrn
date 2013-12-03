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
 * \file nmranet_if.h
 * This file defines the attributes for an NMRAnet interface.
 *
 * @author Stuart W. Baker
 * @date 13 August 2012
 */

#ifndef _nmranet_if_h_
#define _nmranet_if_h_

#include <sys/types.h>
#include <stdint.h>
#include "nmranet_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Number of receive CAN messages that are buffered in the CAN driver.
 */
extern const size_t CAN_RX_BUFFER_SIZE;

/** Number of transmit CAN messages that are buffered in the CAN driver.
 */
extern const size_t CAN_TX_BUFFER_SIZE;

/** Number of receive characters that are buffered in the serial driver.
 */
extern const size_t SERIAL_RX_BUFFER_SIZE;

/** Number of transmit characters that are buffered in the serial driver.
 */
extern const size_t SERIAL_TX_BUFFER_SIZE;

typedef void* nmranet_if_t; /**< interface handle */

/** Known Message type indicators.
 */
enum mti_value {
    MTI_INITIALIZATION_COMPLETE = 0x0100,  /**< initialization complete */
    MTI_VERIFY_NODE_ID_ADDRESSED = 0x0488, /**< verify a Node ID */
    MTI_VERIFY_NODE_ID_GLOBAL = 0x0490,    /**< verify a Node ID globally */
    MTI_VERIFIED_NODE_ID_NUMBER
    = 0x0170, /**< respond to a verify Node ID request */
    MTI_OPTIONAL_INTERACTION_REJECTED = 0x0068, /**< rejected request */
    MTI_TERMINATE_DUE_TO_ERROR = 0x00A8, /**< terminate due to some error */
    MTI_PROTOCOL_SUPPORT_INQUIRY
    = 0x0828,                            /**< inquire on supported protocols */
    MTI_PROTOCOL_SUPPORT_REPLY = 0x0668, /**< reply with supported protocols */
    MTI_CONSUMER_IDENTIFY = 0x08F4,      /**< query about consumers */
    MTI_CONSUMER_IDENTIFIED_RANGE
    = 0x04A4, /**< query about a range of consumers */
    MTI_CONSUMER_IDENTIFIED_UNKNOWN
    = 0x04C7, /**< consumer broadcast, validity unknown */
    MTI_CONSUMER_IDENTIFIED_VALID
    = 0x04C4, /**< consumer broadcast, valid state */
    MTI_CONSUMER_IDENTIFIED_INVALID
    = 0x04C5, /**< consumer broadcast, invalid state */
    MTI_CONSUMER_IDENTIFIED_RESERVED = 0x04C6, /**< reserved for future use */
    MTI_PRODUCER_IDENTIFY = 0x0914,            /**< query about producers */
    MTI_PRODUCER_IDENTIFIED_RANGE
    = 0x0524, /**< query about a range of producers */
    MTI_PRODUCER_IDENTIFIED_UNKNOWN
    = 0x0547, /**< producer broadcast, validity unknown */
    MTI_PRODUCER_IDENTIFIED_VALID
    = 0x0544, /**< producer broadcast, valid state */
    MTI_PRODUCER_IDENTIFIED_INVALID
    = 0x0545, /**< producer broadcast, invalid state */
    MTI_PRODUCER_IDENTIFIED_RESERVED = 0x0546, /**< reserved for future use */
    MTI_EVENTS_IDENTIFY_ADDRESSED
    = 0x0968, /**< request identify all of a node's events */
    MTI_EVENTS_IDENTIFY_GLOBAL
    = 0x0970, /**< request identify all of every node's events */
    MTI_LEARN_EVENT = 0x0594,        /**< */
    MTI_EVENT_REPORT = 0x05B4,       /**< */
    MTI_XPRESSNET = 0x09C0,          /**< */
    MTI_IDENT_INFO_REQUEST = 0x0DE8, /**< request node identity */
    MTI_IDENT_INFO_REPLY = 0x0A08,   /**< node identity reply */
    MTI_DATAGRAM = 0x1C48,           /**< datagram */
    MTI_DATAGRAM_OK = 0x0A28,        /**< datagram received okay */
    MTI_DATAGRAM_REJECTED = 0x0A48,  /**< datagram rejected by receiver */
    MTI_STREAM_DATA = 0x1F88,        /**< stream data */
};

#define MTI_MODIFIER_MASK 0x0003 /**< modifier within Priority/Type mask */
#define MTI_EVENT_MASK 0x0004    /**< event number present mask */
#define MTI_ADDRESS_MASK 0x0008  /**< Address present mask */
#define MTI_SIMPLE_MASK 0x0010   /**< simple protocol mask */
#define MTI_TYPE_MASK 0x03e0     /**< type within priority mask */
#define MTI_PRIORITY_MASK 0x0c00 /**< priority mask */
#define MTI_DATAGRAM_MASK 0x1000 /**< stream or datagram mask */
#define MTI_SPECIAL_MASK 0x2000  /**< special mask */
#define MTI_RESERVED_MASK 0xc000 /**< reserved mask */

#define MTI_MODIFIER_SHIFT 0  /**< modifier within Priority/Type shift */
#define MTI_EVENT_SHIFT 2     /**< event number present shift */
#define MTI_ADDRESS_SHIFT 3   /**< Address present shift */
#define MTI_SIMPLE_SHIFT 4    /**< simple protocol shift */
#define MTI_TYPE_SHIFT 5      /**< type within priority shift */
#define MTI_PRIORITY_SHIFT 10 /**< priority shift */
#define MTI_DATAGRAM_SHIFT 12 /**< stream or datagram shift */
#define MTI_SPECIAL_SHIFT 13  /**< special shift */
#define MTI_RESERVED_SHIFT 14 /**< reserved shift */

/** Get the MTI modifier value field.
 * @param _mti MTI to extract field value from
 * @return extracted MTI field value
 */
#define GET_MTI_MODIFIER(_mti)                                                 \
    (((_mti) & MTI_MODIFIER_MASK) >> MTI_MODIFIER_SHIFT)

/** Get the MTI event value field.
 * @param _mti MTI to extract field value from
 * @return extracted MTI field value
 */
#define GET_MTI_EVENT(_mti) (((_mti) & MTI_EVENT_MASK) >> MTI_EVENT_SHIFT)

/** Get the MTI address value field.
 * @param _mti MTI to extract field value from
 * @return extracted MTI field value
 */
#define GET_MTI_ADDRESS(_mti) (((_mti) & MTI_ADDRESS_MASK) >> MTI_ADDRESS_SHIFT)

/** Get the MTI simple value field.
 * @param _mti MTI to extract field value from
 * @return extracted MTI field value
 */
#define GET_MTI_SIMPLE(_mti) (((_mti) & MTI_SIMPLE_MASK) >> MTI_SIMPLE_SHIFT)

/** Get the MTI type value field.
 * @param _mti MTI to extract field value from
 * @return extracted MTI field value
 */
#define GET_MTI_TYPE(_mti) (((_mti) & MTI_TYPE_MASK) >> MTI_TYPE_SHIFT)

/** Get the MTI priority value field.
 * @param _mti MTI to extract field value from
 * @return extracted MTI field value
 */
#define GET_MTI_PRIORITY(_mti)                                                 \
    (((_mti) & MTI_PRIORITY_MASK) >> MTI_PRIORITY_SHIFT)

/** Get the MTI datagram value field.
 * @param _mti MTI to extract field value from
 * @return extracted MTI field value
 */
#define GET_MTI_DATAGRAM(_mti)                                                 \
    (((_mti) & MTI_DATAGRAM_MASK) >> MTI_DATAGRAM_SHIFT)

/** Get the MTI special value field.
 * @param _mti MTI to extract field value from
 * @return extracted MTI field value
 */
#define GET_MTI_SPECIAL(_mti) (((_mti) & MTI_SPECIAL_MASK) >> MTI_SPECIAL_SHIFT)

/** Set the MTI modifier value field.
 * @param _mti MTI to set field value in
 * @param _modifier value to set field to
 */
#define SET_MTI_MODIFIER(_mti, _modifier)                                      \
    {                                                                          \
        (_mti) &= ~MTI_MODIFIER_MASK;                                          \
        (_mti) |= (_modifier) << MTI_MODIFIER_SHIFT;                           \
    }

/** Set the MTI event value field.
 * @param _mti MTI to set field value in
 * @param _event value to set field to
 */
#define SET_MTI_EVENT(_mti, _event)                                            \
    {                                                                          \
        (_mti) &= ~MTI_EVENT_MASK;                                             \
        (_mti) |= (_event) << MTI_EVENT_SHIFT;                                 \
    }

/** Set the MTI address value field.
 * @param _mti MTI to set field value in
 * @param _address value to set field to
 */
#define SET_MTI_ADDRESS(_mti, _address)                                        \
    {                                                                          \
        (_mti) &= ~MTI_ADDRESS_MASK;                                           \
        (_mti) |= (_address) << MTI_ADDRESS_SHIFT;                             \
    }

/** Set the MTI simple value field.
 * @param _mti MTI to set field value in
 * @param _simple value to set field to
 */
#define SET_MTI_SIMPLE(_mti, _simple)                                          \
    {                                                                          \
        ` (_mti) &= ~MTI_SIMPLE_MASK;                                          \
        (_mti) |= (_simple) << MTI_SIMPLE_SHIFT;                               \
    }

/** Set the MTI type value field.
 * @param _mti MTI to set field value in
 * @param _type value to set field to
 */
#define SET_MTI_TYPE(_mti, _type)                                              \
    {                                                                          \
        (_mti) &= ~MTI_TYPE_MASK;                                              \
        (_mti) |= (_type) << MTI_TYPE_SHIFT;                                   \
    }

/** Set the MTI priority value field.
 * @param _mti MTI to set field value in
 * @param _priority value to set field to
 */
#define SET_MTI_PRIORITY(_mti, _priority)                                      \
    {                                                                          \
        (_mti) &= ~MTI_PRIORITY_MASK;                                          \
        (_mti) |= (_priority) << MTI_PRIORITY_SHIFT;                           \
    }

/** Set the MTI datagram value field.
 * @param _mti MTI to set field value in
 * @param _datagram value to set field to
 */
#define SET_MTI_DATAGRAM(_mti, _datagram)                                      \
    {                                                                          \
        (_mti) &= ~MTI_DATAGRAM_MASK;                                          \
        (_mti) |= (_datagram) << MTI_DATAGRAM_SHIFT;                           \
    }

/** Set the MTI special value field.
 * @param _mti MTI to set field value in
 * @param _special value to set field to
 */
#define SET_MTI_SPECIAL(_mti, _special)                                        \
    {                                                                          \
        (_mti) &= ~MTI_SPECIAL_MASK;                                           \
        (_mti) |= (_special) << MTI_SPECIAL_SHIFT;                             \
    }

/** Set all the MTI value fields.
 * @param _mti MTI to set field values in
 * @param _modifier value to set field to
 * @param _event value to set field to
 * @param _address value to set field to
 * @param _simple value to set field to
 * @param _type value to set field to
 * @param _priority value to set field to
 * @param _datagram value to set field to
 * @param _special value to set field to
 */
#define SET_MTI_FIELDS(_mti, _modifier, _event, _address, _simple, _type,      \
                       _priority, _datagram, _special)                         \
    {                                                                          \
        (_mti) = ((_modifier) << MTI_MODIFIER_SHIFT)                           \
                 + ((_event) << MTI_EVENT_SHIFT)                               \
                 + ((_address) << MTI_ADDRESS_SHIFT)                           \
                 + ((_simple) << MTI_SIMPLE_SHIFT)                             \
                 + ((_type) << MTI_TYPE_SHIFT)                                 \
                 + ((_priority) << MTI_PRIORITY_SHIFT)                         \
                 + ((_datagram) << MTI_DATAGRAM_SHIFT)                         \
                 + ((_special) << MTI_SPECIAL_SHIFT);                          \
    }

/** Information about the interface */
typedef struct nmranet_if
{
    const char* name; /**< name of interface */
    /** method for putting data onto interface */
    int (*write)(struct nmranet_if* nmranet_if, uint16_t mti, node_id_t src,
                 node_handle_t dst, const void* data);
    /** Reentrant method to lookup a 48-bit Node ID from a given alias */
    node_id_t (*lookup_id)(struct nmranet_if*, node_id_t, node_alias_t);
    void* priv; /**< private data for upper layer use */
} NMRAnetIF;

/** This method can be called by any interface to indicate it has incoming data.
 */
void nmranet_if_rx_data(struct nmranet_if* nmranet_if, uint16_t mti,
                        node_handle_t src, node_id_t dst, const void* data);

/** Initialize the network stack.
 * @param node_id Node ID used to identify the built in bridge, 0 for no bridge
 */
void nmranet_init(node_id_t node_id);

/** Initialize a NMRAnet interface.
 * @param nmranet_if instance of this interface
 */
void nmranet_if_init(NMRAnetIF* nmranet_if);

/** Initialize a can interface.
 * @param node_id node ID of interface
 * @param device description for this instance
 * @param if_read read method for this interface
 * @param if_write write method for this interface
 * @return handle to the NMRAnet interface
 */
NMRAnetIF* nmranet_can_if_init(node_id_t node_id, const char* device,
                               ssize_t (*if_read)(int, void*, size_t),
                               ssize_t (*if_write)(int, const void*, size_t));

/** Initialize a Grid Connect interface.
 * @param node_id node ID of interface
 * @param device description for this instance
 * @return handle to the NMRAnet interface
 */
NMRAnetIF* nmranet_gc_if_init(node_id_t node_id, const char* device);

/** Send an ident info reply message.
 * @param node_id Node ID to respond as
 * @param dst destination Node ID to respond to
 * @param description index 0 name of manufacturer
 *                    index 1 name of model
 *                    index 2 hardware revision
 *                    index 3 software revision
 */
void nmranet_if_ident_info_reply(node_id_t node_id, node_id_t dst,
                                 const char* description[4]);

/** Get the local interface instance.
 * @return local interface instance
 */
NMRAnetIF* nmranet_lo_if(void);

#ifdef __cplusplus
}
#endif

#endif /* _nmranet_if_h_ */
