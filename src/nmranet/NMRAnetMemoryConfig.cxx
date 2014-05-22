/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file NMRAnetMemoryConfig.cxx
 * This file defines NMRAnet Memory Configuration Protocol(s).
 *
 * @author Stuart W. Baker
 * @date 13 October 2013
 */

#include "nmranet/NMRAnetMemoryConfig.hxx"
#include "nmranet/NMRAnetNode.hxx"

namespace nmranet
{

class Node;

/* default CDI if it is not overriden by the application */
const uint8_t __attribute__ ((weak)) MemoryConfig::globalCdi[] = { 0 };

size_t MemoryConfig::globalCdiSize = strlen((char*)globalCdi) + 1;

/** Default constructor.
 * @param cdi Configuration Data, else NULL to use default CDI
 */
MemoryConfig::MemoryConfig(uint8_t *cdi)
    : cdi(cdi ? cdi : globalCdi),
      cdiSize(cdi ? strlen((char*)cdi) + 1 : globalCdiSize)
{
}

/** Default hander for memory config all.
 * @param address offset in bytes to grab
 * @param count number of bytes to gram [in], number of bytes available [out]
 * @return pointer in memory to address space + offset
 */
__attribute__ ((weak))
void *MemoryConfig::all_memory(uint32_t address, uint8_t *count)
{
    return (char*)0 + address;
}

/** Helper function for replyoing to a memory config request.
 * @param from node we are replying to
 * @param command configuration command
 * @param address address within address space the data coresponds to
 * @param data location in memory to copy address space data from
 * @param count number of bytes to send
 */
void MemoryConfig::reply(NodeHandle from,
                         const void *command, const void *address,
                         const void *data, const uint8_t count)
{
    Node *node = static_cast<Node*>(this);

    Buffer *buffer = node->buffer_get(Datagram::CONFIGURATION, count + 5, OS_WAIT_FOREVER);
    node->buffer_fill(buffer, Datagram::CONFIGURATION, command, COMMAND, 1);
    node->buffer_fill(buffer, Datagram::CONFIGURATION, address, ADDRESS, sizeof(uint32_t));
    if (count)
    {
        node->buffer_fill(buffer, Datagram::CONFIGURATION, data, DATA, count);
    }
    node->produce(from, buffer, OS_WAIT_FOREVER);
}

/** Process a memory configuration datagram.
 * @param datagram datagram to process, this pointer is stale upon return
 */
void MemoryConfig::process(Buffer *buffer)
{
    Datagram::Message *m = (Datagram::Message*)buffer->start();
    Node *node = static_cast<Node*>(this);

    uint8_t *data = Datagram::payload(buffer);
    uint8_t space;
    uint32_t address;
    uint8_t space_offset = 0;
    uint8_t space_special = data[COMMAND] & COMMAND_FLAG_MASK;

    /* figure out what memory space we are operating on */
    switch (space_special)
    {
        default:
            /* This is not a special memory space */
            space = data[SPACE];
            space_offset = 1;
            break;
        case COMMAND_CDI:        /* fall through */
        case COMMAND_ALL_MEMORY: /* fall through */
        case COMMAND_CONFIG:
            space = SPACE_SPECIAL + space_special;
            break;
    }
    
    /* figure out the address */
    address = ((uint32_t)data[ADDRESS + 0] << 24) +
              ((uint32_t)data[ADDRESS + 1] << 16) +
              ((uint32_t)data[ADDRESS + 2] <<  8) +
              ((uint32_t)data[ADDRESS + 3] <<  0);
              
    /* figure out the command */
    switch (data[COMMAND] & COMMAND_MASK)
    {
        default:
            /* We don't know this command */
            break;
        case COMMAND_READ:
        {
            uint8_t count = data[COUNT + space_offset] & COUNT_MASK;
            HASSERT(count <= 64);
            uint8_t command = COMMAND_READ_REPLY;
            if (space == SPACE_CDI)
            {
                command |= COMMAND_CDI;

                /* adjust count for size of CDI space */
                if (address > cdiSize)
                {
                    count = 0;
                }
                else
                {
                    count = (address + count) < cdiSize ?
                            count : (cdiSize - address);
                }

                reply(m->from, &command, &data[ADDRESS], cdi + address, count);
            }
            else if (space == SPACE_ALL_MEMORY)
            {
                void *start = all_memory(address, &count);
                command |= COMMAND_ALL_MEMORY;
                reply(m->from, &command, &data[ADDRESS], start, count);
            }
            else if (space == SPACE_CONFIG)
            {
                command |= COMMAND_CONFIG;
                reply(m->from, &command, &data[ADDRESS], (char*)0 + address, count);
            }
            break;
        }
        case COMMAND_WRITE:
        {
            uint8_t count = data[COUNT + space_offset] & COUNT_MASK;
            HASSERT(count <= 64);
            //uint8_t command = COMMAND_WRITE_REPLY;
            if (space == SPACE_CDI)
            {
                /* CDI is read only */
                HASSERT(1);
            }
            else if (space == SPACE_ALL_MEMORY)
            {
                void *start = all_memory(address, &count);
                size_t size = m->size - (6 + space_offset);
                memcpy(start, &data[DATA + space_offset], size);
                //command |= COMMAND_ALL_MEMORY;
                //reply(m->from, &command, &data[ADDRESS], NULL, 0);
            }
            else if (space == SPACE_CONFIG)
            {
                //command |= COMMAND_CONFIG;
                //reply(m->from, &command, &data[ADDRESS], NULL, 0);
            }
            break;
        }
        case COMMAND_WRITE_UNDER_MASK:
        {
            uint8_t count = data[COUNT + space_offset] & COUNT_MASK;
            HASSERT(count <= 64);
            //uint8_t command = COMMAND_WRITE_REPLY;
            if (space == SPACE_CDI)
            {
                /* CDI is read only */
                HASSERT(1);
            }
            else if (space == SPACE_ALL_MEMORY)
            {
                uint8_t *start = (uint8_t*)all_memory(address, &count);
                uint8_t *from = &data[DATA + space_offset];
                size_t size = m->size - (6 + space_offset);
                for (unsigned int i = 0, j = 0; i < size; ++i, j += 2)
                {
                    start[i] &= ~from[j];
                    start[i] |= (from[j] & from[j+1]);
                }
                //command |= COMMAND_ALL_MEMORY;
                //reply(m->from, &command, &data[ADDRESS], NULL, 0);
            }
            else if (space == SPACE_CONFIG)
            {
                //command |= COMMAND_CONFIG;
                //reply(m->from, &command, &data[ADDRESS], NULL, 0);
            }
            break;
        }
        case COMMAND_OPTIONS:
        {
            uint16_t available = AVAIL_WUM | AVAIL_UR | AVAIL_UW;
            uint8_t reply[6];
            reply[COMMAND] = COMMAND_OPTIONS_REPLY;
            reply[AVAILABLE] = available >> 8;
            reply[AVAILABLE + 1] = available & 0xFF;
            reply[LENGTHS] = 0;
            reply[HIGHEST] = SPACE_CDI;
            reply[LOWEST] = SPACE_CONFIG;
            node->produce(m->from, Datagram::CONFIGURATION, reply, 6, OS_WAIT_FOREVER);
            break;
        }
        case COMMAND_INFORMATION:
        {
            uint32_t address_highest;
            uint8_t reply[7];
            reply[COMMAND] = COMMAND_INFORMATION_REPLY;
            reply[FLAGS] = 0;
            if (data[ADDRESS_SPACE] == SPACE_CDI)
            {
                address_highest = cdiSize - 1;
                reply[COMMAND] |= COMMAND_PRESENT;
                reply[FLAGS] = FLAG_RO;
            }
            else if (data[ADDRESS_SPACE] == SPACE_ALL_MEMORY)
            {
                address_highest = UINTPTR_MAX;
                reply[COMMAND] |= COMMAND_PRESENT;
            }
            else if (data[ADDRESS_SPACE] == SPACE_CONFIG)
            {
                address_highest = 0;
            }
            else
            {
                address_highest = 0;
            }
            reply[ADDRESS_SPACE] = data[ADDRESS_SPACE];
            reply[ADDRESS_HIGHEST + 0] = (address_highest >> 24) & 0xFF;
            reply[ADDRESS_HIGHEST + 1] = (address_highest >> 16) & 0xFF;
            reply[ADDRESS_HIGHEST + 2] = (address_highest >>  8) & 0xFF;
            reply[ADDRESS_HIGHEST + 3] = (address_highest >>  0) & 0xFF;
            node->produce(m->from, Datagram::CONFIGURATION, reply, 7, OS_WAIT_FOREVER);
            break;
        }
    }
    node->received_okay(buffer, Datagram::FLAGS_NONE);
}

};

