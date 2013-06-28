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
 * \file nmranet_can_if.h
 * This file defines an NMRAnet interface for generic CAN.  This is a private
 * header file and should only be included in nmranet_can_if.c.
 *
 * @author Stuart W. Baker
 * @date 13 August 2012
 */

#ifndef _nmranet_can_if_h_
#define _nmranet_can_if_h_

#include "if/nmranet_if.h"

#define CONTROL_FRAME_SEQUENCE_MASK 0x7000000 /**< sequence mask for a control frame */
#define RID_FRAME 0x0700 /**< Reserve ID Frame */
#define AMD_FRAME 0x0701 /**< Alias Map Definition frame */
#define AME_FRAME 0x0702 /**< Alias Mapping Inquery */
#define AMR_FRAME 0x0703 /**< Alias Map Reset */

#define CAN_ID_SOURCE_MASK         0x00000fff /**< mask for source field of CAN ID */
#define CAN_ID_MTI_MASK            0x00fff000 /**< mask for MTI field of CAN ID */
#define CAN_ID_DST_MASK            0x00fff000 /**< mask for MTI field of CAN ID */
#define CAN_ID_CAN_FRAME_TYPE_MASK 0x07000000 /**< mask for can frame type field of CAN ID */
#define CAN_ID_FRAME_TYPE_MASK     0x08000000 /**< mask for frame type field of CAN ID */
#define CAN_ID_PRIORITY_MASK       0x10000000 /**< mask for priority field of CAN ID */
#define CAN_ID_PADDING_MASK        0xe0000000 /**< mask for padding field of CAN ID */

#define CAN_ID_SOURCE_SHIFT          0 /**< shift for source field of CAN ID */
#define CAN_ID_MTI_SHIFT            12 /**< shift for MTI field of CAN ID */
#define CAN_ID_DST_SHIFT            12 /**< shift for MTI field of CAN ID */
#define CAN_ID_CAN_FRAME_TYPE_SHIFT 24 /**< shift for can frame type field of CAN ID */
#define CAN_ID_FRAME_TYPE_SHIFT     27 /**< shift for frame type field of CAN ID */
#define CAN_ID_PRIORITY_SHIFT       28 /**< shift for priority field of CAN ID */
#define CAN_ID_PADDING_SHIFT        29 /**< shift for padding field of CAN ID */

/** Get the source field value of the CAN ID.
 * @param _can_id identifier to act upon
 * @return source field
 */
#define GET_CAN_ID_SOURCE(_can_id) \
    (((_can_id) & CAN_ID_SOURCE_MASK) >> CAN_ID_SOURCE_SHIFT)

/** Get the MTI field value of the CAN ID.
 * @param _can_id identifier to act upon
 * @return MTI field value
 */
#define GET_CAN_ID_MTI(_can_id) \
    (((_can_id) & CAN_ID_MTI_MASK) >> CAN_ID_MTI_SHIFT)

/** Get the destination field value of the CAN ID.
 * @param _can_id identifier to act upon
 * @return destination field value
 */
#define GET_CAN_ID_DST(_can_id) \
    (((_can_id) & CAN_ID_DST_MASK) >> CAN_ID_DST_SHIFT)

/** Get the CAN frame type field value of the CAN ID.
 * @param _can_id identifier to act upon
 * @return CAN frame type field value
 */
#define GET_CAN_ID_CAN_FRAME_TYPE(_can_id) \
    (((_can_id) & CAN_ID_CAN_FRAME_TYPE_MASK) >> CAN_ID_CAN_FRAME_TYPE_SHIFT)

/** Get the frame type field value of the CAN ID.
 * @param _can_id identifier to act upon
 * @return frame type field value
 */
#define GET_CAN_ID_FRAME_TYPE(_can_id) \
    (((_can_id) & CAN_ID_FRAME_TYPE_MASK) >> CAN_ID_FRAME_TYPE_SHIFT)

/** Get the priority field value of the CAN ID.
 * @param _can_id identifier to act upon
 * @return priority field value
 */
#define GET_CAN_ID_PRIORITY(_can_id) \
    (((_can_id) & CAN_ID_PRIORITY_MASK) >> CAN_ID_PRIORITY_SHIFT)

/** Set the source field value of the CAN ID.
 * @param _can_id identifier to act upon
 * @param _source source field value
 */
#define SET_CAN_ID_SOURCE(_can_id, _source)        \
{                                                  \
    (_can_id) &= ~CAN_ID_SOURCE_MASK;              \
    (_can_id) |= (_source) << CAN_ID_SOURCE_SHIFT; \
}

/** Set the MTI field value of the CAN ID.
 * @param _can_id identifier to act upon
 * @param _mti MTI field value
 */
#define SET_CAN_ID_MTI(_can_id, _mti)        \
{                                            \
    (_can_id) &= ~CAN_ID_MTI_MASK;           \
    (_can_id) |= (_mti) << CAN_ID_MTI_SHIFT; \
}

/** Set the destination field value of the CAN ID.
 * @param _can_id identifier to act upon
 * @param _dst destination field value
 */
#define SET_CAN_ID_DST(_can_id, _dst)        \
{                                            \
    (_can_id) &= ~CAN_ID_DST_MASK;           \
    (_can_id) |= (_dst) << CAN_ID_DST_SHIFT; \
}

/** Set the CAN frame type field value of the CAN ID.
 * @param _can_id identifier to act upon
 * @param _type CAN frame type field value
 */
#define SET_CAN_ID_CAN_FRAME_TYPE(_can_id, _type)        \
{                                                        \
    (_can_id) &= ~CAN_ID_CAN_FRAME_TYPE_MASK;            \
    (_can_id) |= (_type) << CAN_ID_CAN_FRAME_TYPE_SHIFT; \
}

/** Set the frame type field value of the CAN ID.
 * @param _can_id identifier to act upon
 * @param _type frame type field value
 */
#define SET_CAN_ID_FRAME_TYPE(_can_id, _type)        \
{                                                    \
    (_can_id) &= ~CAN_ID_FRAME_TYPE_MASK;            \
    (_can_id) |= (_type) << CAN_ID_FRAME_TYPE_SHIFT; \
}

/** Set the priority field value of the CAN ID.
 * @param _can_id identifier to act upon
 * @param _priority pryority field value
 */
#define SET_CAN_ID_PRIORITY(_can_id, _priority)        \
{                                                      \
    (_can_id) &= ~CAN_ID_PRIORITY_MASK;                \
    (_can_id) |= (_priority) << CAN_ID_PRIORITY_SHIFT; \
}

/** Set all the CAN ID fields.
 * @param _can_id identifier to act upon
 * @param _source source field value
 * @param _mti MTI field value
 * @param _can_type CAN frame type field value
 * @param _type frame type field value
 * @param _priority priority field value
 */
#define SET_CAN_ID_FIELDS(_can_id, _source, _mti, _can_type, _type, _priority) \
{                                                                              \
    (_can_id) = ((_source)   << CAN_ID_SOURCE_SHIFT        ) +                 \
                ((_mti)      << CAN_ID_MTI_SHIFT           ) +                 \
                ((_can_type) << CAN_ID_CAN_FRAME_TYPE_SHIFT) +                 \
                ((_type)     << CAN_ID_FRAME_TYPE_SHIFT    ) +                 \
                ((_priority) << CAN_ID_PRIORITY_SHIFT      );                  \
}

#define CAN_CONTROL_FRAME_SOURCE_MASK   0x00000fff /**< source alias mask */
#define CAN_CONTROL_FRAME_FIELD_MASK    0x00fff000 /**< control field data mask */
#define CAN_CONTROL_FRAME_SEQUENCE_MASK 0x07000000 /**< frame sequence number mask */
#define CAN_CONTROL_FRAME_TYPE_MASK     0x08000000 /**< value of '0' means control frame mask */
#define CAN_CONTROL_FRAME_PRIORITY_MASK 0x10000000 /**< priority mask */
#define CAN_CONTROL_FRAME_PADDING_MASK  0xe0000000 /**< pad out to a full 32-bit word */

#define CAN_CONTROL_FRAME_SOURCE_SHIFT    0 /**< source alias shift */
#define CAN_CONTROL_FRAME_FIELD_SHIFT    12 /**< control field data shift */
#define CAN_CONTROL_FRAME_SEQUENCE_SHIFT 24 /**< frame sequence number shift */
#define CAN_CONTROL_FRAME_TYPE_SHIFT     27 /**< value of '0' means control frame shift */
#define CAN_CONTROL_FRAME_PRIORITY_SHIFT 28 /**< priority shift */
#define CAN_CONTROL_FRAME_PADDING_SHIFT  29 /**< pad out to a full 32-bit word */

/** Get the source field of the a can control frame.
 * @param _can_id CAN ID of the control frame
 * @return value of the source field
 */
#define GET_CAN_CONTROL_FRAME_SOURCE(_can_id) \
    (((_can_id) & CAN_CONTROL_FRAME_SOURCE_MASK) >> CAN_CONTROL_FRAME_SOURCE_SHIFT)
    
/** Get the field field of the a can control frame.
 * @param _can_id CAN ID of the control frame
 * @return value of the field field
 */
#define GET_CAN_CONTROL_FRAME_FIELD(_can_id) \
    (((_can_id) & CAN_CONTROL_FRAME_FIELD_MASK) >> CAN_CONTROL_FRAME_FIELD_SHIFT)
    
/** Get the sequence field of the a can control frame.
 * @param _can_id CAN ID of the control frame
 * @return value of the sequence field
 */
#define GET_CAN_CONTROL_FRAME_SEQUENCE(_can_id) \
    (((_can_id) & CAN_CONTROL_FRAME_SEQUENCE_MASK) >> CAN_CONTROL_FRAME_SEQUENCE_SHIFT)
    
/** Set the source field of the a can control frame.
 * @param _can_id CAN ID of the control frame
 * @param _source value to set
 */
#define SET_CAN_CONTROL_FRAME_SOURCE(_can_id, _source)        \
{                                                             \
    (_can_id) &= ~CAN_CONTROL_FRAME_SOURCE_MASK);             \
    (_can_id) |= (_source) << CAN_CONTROL_FRAME_SOURCE_SHIFT; \
}

/** Set the field field of the a can control frame.
 * @param _can_id CAN ID of the control frame
 * @param _field value to set
 */
#define SET_CAN_CONTROL_FRAME_FIELD(_can_id, _field)        \
{                                                           \
    (_can_id) &= ~CAN_CONTROL_FRAME_FIELD_MASK);            \
    (_can_id) |= (_field) << CAN_CONTROL_FRAME_FIELD_SHIFT; \
}

/** Set the sequence field of the a can control frame.
 * @param _can_id CAN ID of the control frame
 * @param _sequence value to set
 */
#define SET_CAN_CONTROL_FRAME_SEQUENCE(_can_id, _sequence)        \
{                                                                 \
    (_can_id) &= ~CAN_CONTROL_FRAME_SEQUENCE_MASK);               \
    (_can_id) |= (_sequence) << CAN_CONTROL_FRAME_SEQUENCE_SHIFT; \
}

/** Initialize a control frame CAN ID and set DLC to 0.
 * @param _frame control frame to initialize
 * @param _source source data
 * @param _field field data
 * @param _sequence sequence data
 */
#define CAN_CONTROL_FRAME_INIT(_frame, _source, _field, _sequence)        \
{                                                                         \
    (_frame).can_id = ((_source)   << CAN_CONTROL_FRAME_SOURCE_SHIFT  ) + \
                      ((_field)    << CAN_CONTROL_FRAME_FIELD_SHIFT   ) + \
                      ((_sequence) << CAN_CONTROL_FRAME_SEQUENCE_SHIFT) + \
                      ((0)         << CAN_CONTROL_FRAME_TYPE_SHIFT    ) + \
                      ((1)         << CAN_CONTROL_FRAME_PRIORITY_SHIFT);  \
    (_frame).can_dlc = 0;                                                 \
}

#if 0
/** Address fields within an addressed frame type.
 */
typedef struct addressed_fields
{
    uint16_t destination : 12; /**< destination alias */
    uint16_t frame       : 2;  /**< 0b00 = only frame, 0b01 = first frame,
                                    0b10 = last frame, 0b11 = middle frame */
    uint16_t reserved    : 2;  /**< reserved for future use */
} AddressedFields;
#endif

#define ADDRESSED_FIELD_DESTINATION_MASK 0x0fff /**< destination alias mask */
/** 0b00 = only frame, 0b01 = first frame,
 *  0b10 = last frame, 0b11 = middle frame (mask)
 */
#define ADDRESSED_FIELD_FRAME_MASK       0x3000 
#define ADDRESSED_FIELD_RESERVED_MASK    0xc000 /**< reserved for future use */

#define ADDRESSED_FIELD_DESTINATION_SHIFT  0 /**< destination alias shift */
/** 0b00 = only frame, 0b01 = first frame,
 *  0b10 = last frame, 0b11 = middle frame (shift)
 */
#define ADDRESSED_FIELD_FRAME_SHIFT       12 
#define ADDRESSED_FIELD_RESERVED_SHIFT    14 /**< reserved for future use */

/** Get the destination field of addressed data.
 * @param _address addressed data
 * @return destination field value
 */
#define GET_ADDRESSED_FIELD_DESTINATION(_address) \
    (((_address) & ADDRESSED_FIELD_DESTINATION_MASK) >> ADDRESSED_FIELD_DESTINATION_SHIFT)

/** Get the frame field of addressed data.
 * @param _address addressed data
 * @return frame field value
 */
#define GET_ADDRESSED_FIELD_FRAME(_address) \
    (((_address) & ADDRESSED_FIELD_FRAME_MASK) >> ADDRESSED_FIELD_FRAME_SHIFT)

/** Set the destination field of addressed data.
 * @param _address addressed data
 * @param _dst destination field value
 */
#define SET_ADDRESSED_FIELD_DESTINATION(_address, _dst)        \
{                                                              \
    (_address) &= ~ADDRESSED_FIELD_DESTINATION_MASK;           \
    (_address) |= (_dst) << ADDRESSED_FIELD_DESTINATION_SHIFT; \
}

/** Set the frame field of addressed data.
 * @param _address addressed data
 * @param _frame destination field value
 */
#define SET_ADDRESSED_FIELD_FRAME(_address, _frame)        \
{                                                          \
    (_address) &= ~ADDRESSED_FIELD_FRAME_MASK;             \
    (_address) |= (_frame) << ADDRESSED_FIELD_FRAME_SHIFT; \
}

/** Set the frame field of addressed data.
 * @param _address addressed data
 * @param _dst destination field value
 * @param _frame destination field value
 */
#define SET_ADDRESSED_FIELDS(_address, _dst, _frame)               \
{                                                                  \
    (_address) = ((_dst)   << ADDRESSED_FIELD_DESTINATION_SHIFT) + \
                 ((_frame) << ADDRESSED_FIELD_FRAME_SHIFT      );  \
}

/** CAN frame types */
enum can_frame_type
{
    TYPE_GLOBAL_ADDRESSED      = 1, /**< global and addressed MTI */
    TYPE_DATAGRAM_ONE_FRAME    = 2, /**< complete datagram in a single frame */
    TYPE_DATAGRAM_FIRST_FRAME  = 3, /**< first frame of a datagram */
    TYPE_DATAGRAM_MIDDLE_FRAME = 4, /**< Middle frame of a datagram */
    TYPE_DATAGRAM_FINAL_FRAME  = 5, /**< Final frame of a datagram */
    TYPE_STREAM_DATA           = 7, /**< Stream data */
};

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize a can interface.
 * @param node_id node ID of interface
 * @param device description for this instance
 * @param if_read read method for this interface
 * @param if_write write method for this interface
 * @return handle to the NMRAnet interface
 */
NMRAnetIF *nmranet_can_if_init(node_id_t node_id, const char *device,
                               ssize_t (*if_read)(int, void*, size_t),
                               ssize_t (*if_write)(int, const void*, size_t));


/** Initialize a can interface.
 * @param node_id node ID of interface
 * @param read_fd file descriptor to read from
 * @param write_fd file descriptor to write to
 * @param thread_name description for this instance
 * @param if_read read method for this interface
 * @param if_write write method for this interface
 * @return handle to the NMRAnet interface
 */
NMRAnetIF *nmranet_can_if_fd_init(node_id_t node_id, int read_fd, int write_fd,
                                  const char* thread_name,
                                  ssize_t (*if_read)(int, void*, size_t),
                                  ssize_t (*if_write)(int, const void*, size_t));

#ifdef __cplusplus
}
#endif

#endif /* _nmranet_can_if_h_ */
