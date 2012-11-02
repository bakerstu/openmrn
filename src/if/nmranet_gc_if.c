/** \copyright
 * Copyright (c) 2012, Stuart W Baker
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
 * \file nmranet_gc_if.h
 * This file defines an NMRAnet interface for Grid Connect based on CAN.
 *
 * @author Stuart W. Baker
 * @date 19 October 2012
 */

#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include "nmranet_can.h"
#include "if/nmranet_can_if.h"

/** Build an ASCII character representation of a nibble value
 * @param value to convert
 * @return converted value
 */
static char nibble_to_ascii(int nibble)
{
    nibble &= 0xf;

    if (nibble < 10)
    {
        return ('0' + nibble);
    }
    
    return ('A' + (nibble - 10));
}

/** Take a pair of ASCII characters and convert them to a byte value.
 * pointer to two ASCII characters
 * @return byte value
 */
static int ascii_pair_to_byte(const char *pair)
{
    unsigned char* data = (unsigned char*)pair;
    int result;
    
    if (data[1] < 'A')
    {
        result = data[1] - '0';
    }
    else
    {
        result = data[1] - 'A' + 10;
    }
    
    if (data[0] < 'A')
    {
        result += (data[0] - '0') << 4;
    }
    else
    {
        result += (data[0] - 'A' + 10) << 4;
    }
    
    return result;
}

/** Write a CAN packet to Grid Connect interface.
 * @param fd file descriptor for device
 * @param data can data to write
 * @param len length of data, should be a multiple of sizeof(struct can_frame)
 * @return number of bytes written, or -1 with errno set
 */
static ssize_t gc_write(int fd, const void *data, size_t len)
{
    struct can_frame *can_frame = (struct can_frame*)data;
    size_t            remaining = len;
    
    /* check for len that is multiple of a can_frame size */
    if ((len % sizeof(struct can_frame)) != 0)
    {
        errno = EINVAL;
        return -1;
    }
    
    /* allocate a buffer for the Grid connect format */
    unsigned char buf[52];
    buf[0] = buf[1] = '!';

    /* while there are packets to transmit */
    while (remaining)
    {
        int index;
        /* handle the identifier */
        if (IS_CAN_FRAME_EFF(*can_frame))
        {
            buf[2] = buf[3] = 'X';
            uint32_t id = GET_CAN_FRAME_ID(*can_frame);
            for (int i = 18; i >= 4; i -= 2, id >>= 4)
            {
                buf[i] = buf[i+1] = nibble_to_ascii(id);
            }
            index = 20;
        }
        else
        {
            buf[2] = buf[3] = 'S';
            uint16_t id = GET_CAN_FRAME_ID(*can_frame);
            for (int i = 8; i >= 4; i -= 2, id >>= 4)
            {
                buf[i] = buf[i+1] = nibble_to_ascii(id);
            }
            index = 10;
        }

        /* handle remote or normal */
        if (IS_CAN_FRAME_RTR(*can_frame))
        {
            buf[index] = buf[index + 1] = 'R';
            index += 2;
        }
        else
        {
            buf[index] = buf[index + 1] = 'N';
            index += 2;
        }
        
        /* handle the data */
        for (int i = 0; i < can_frame->can_dlc; i++, index += 4)
        {
            buf[index + 0] = buf[index + 1] = nibble_to_ascii(can_frame->data[i] >> 4);
            buf[index + 2] = buf[index + 3] = nibble_to_ascii(can_frame->data[i]);
        }
        
        /* stop character */
        buf[index] = buf[index + 1] = ';';
        index += 2;
        
        /* write the formated packet */
        int result = write(fd, buf, index);
        
        if (result != index)
        {
            return -1;
        }

        /* get ready for next packet */
        remaining -= sizeof(struct can_frame);
        can_frame++;
    } /* while (remaining) */

    return len;
}

/** Read a CAN packet to Grid Connect interface.
 * @param fd file descriptor for device
 * @param data can data to read
 * @param len length of data, should be a multiple of sizeof(struct can_frame)
 * @return number of bytes read, or -1 with errno set
 */
static ssize_t gc_read(int fd, void *data, size_t len)
{
    struct can_frame *can_frame = (struct can_frame*)data;
    size_t            remaining = len;
    
    /* check for len that is multiple of a can_frame size */
    if ((len % sizeof(struct can_frame)) != 0)
    {
        errno = EINVAL;
        return -1;
    }

    /* while there are packets to receive */
    while (remaining)
    {
        char buf[40];

        /** @todo this decode method is simple, but could be optimized. */
        for ( ; /* until we find a CAN frame */ ; )
        {
            do
            {
                int result = read(fd, buf, 1);
                if (result < 1)
                {
                    return -1;
                }
            } while(buf[0] != ':');
            
            /* We have a start of frame, now lets get the rest of the packet */
            int i;
            for (i = 1; i < 40; i++)
            {
                int result = read(fd, buf + i, 1);
                if (result < 1)
                {
                    return -1;
                }
                if (buf[i] == ';')
                {
                    /* found an end of frame */
                    break;
                }
            }
            if (i != 40)
            {
                /* we found the end of frame character */
                break;
            }
        } /* for ( ; until we find a CAN frame ; ) */
        
        int index;
        /* determine if the frame is standard or extended */
        if (buf[1] == 'X')
        {
            SET_CAN_FRAME_EFF(*can_frame);
            uint32_t id;
            id  = ascii_pair_to_byte(buf + 2) << 24;
            id += ascii_pair_to_byte(buf + 4) << 16;
            id += ascii_pair_to_byte(buf + 6) << 8;
            id += ascii_pair_to_byte(buf + 8) << 0;
            SET_CAN_FRAME_ID_EFF(*can_frame, id);
            index = 10;
        }
        else if (buf[1] == 'S')
        {
            CLR_CAN_FRAME_EFF(*can_frame);
            uint16_t id;
            id  = ascii_pair_to_byte(buf + 2) << 4;
            id |= ascii_pair_to_byte(buf + 3) << 0;
            SET_CAN_FRAME_ID(*can_frame, id);
            index = 5;
        }
        else
        {
            /* unexpected character, try again */
            continue;
        }
        
        /* determine if the frame is normal or remote */
        if (buf[index] == 'N')
        {
            CLR_CAN_FRAME_RTR(*can_frame);
        }
        else if (buf[index] == 'R')
        {
            SET_CAN_FRAME_RTR(*can_frame);
        }
        else
        {
            /* unexpected character, try again */
            continue;
        }
        index++;
        
        /* grab the data */
        int i;
        for (i = 0; buf[index] != ';'; index += 2, i++)
        {
            can_frame->data[i] = ascii_pair_to_byte(buf + index);
        }
        can_frame->can_dlc = i;

        CLR_CAN_FRAME_ERR(*can_frame);
        
        /* get ready for next packet */
        remaining -= sizeof(struct can_frame);
        can_frame++;
    } /* while (remaining) */
    return len;
}

/** Initialize a Grid Connect interface.
 * @param node_id node ID of interface
 * @param device description for this instance
 */
void nmranet_gc_if_init(node_id_t node_id, const char *device)
{
    nmranet_can_if_init(node_id, device, gc_read, gc_write);
}

