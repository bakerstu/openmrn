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
 * \file nmranet_memory_config.c
 * This file defines the handling of memory configuration spaces.
 *
 * @author Stuart W. Baker
 * @date 21 July 2013
 */

#include <stdlib.h>
#include <assert.h>

#include "core/nmranet_datagram_private.h"
#include "nmranet_configuration.h"
#include "os/os.h"

/* default CDI if it is not overriden by the application */
const uint8_t __attribute__((weak)) cdi[] = {0};

/* CDI instance prototype */
extern const uint8_t cdi[];

#include <stdio.h>

/** Size of CDI data in bytes, initialized at startup */
size_t cdiSize = 0;

/** One time initialization. */
void nmranet_memory_config_init(void) { cdiSize = strlen((char *)cdi) + 1; }

/** Default hander for memory config all.
 * @param node node to act on
 * @param address offset in bytes to grab
 * @param count number of bytes to gram [in], number of bytes available [out]
 * @return pointer in memory to address space + offset
 */
void *nmranet_memory_config_all(node_t node, uint32_t address, uint8_t *count)
    __attribute__((weak));
void *nmranet_memory_config_all(node_t node, uint32_t address, uint8_t *count) {
  return (char *)0 + address;
}

void *nmranet_memory_config_all(node_t node, uint32_t address, uint8_t *count);

/** Helper function for replyoing to a memory config request.
 * @param node node making the reply
 * @param from node we are replying to
 * @param command configuration command
 * @param address address within address space the data coresponds to
 * @param data location in memory to
 copy address space data from
 * @param count number of bytes to send
 */
static void memory_config_reply(node_t node, node_handle_t from,
                                const void *command, const void *address,
                                const void *data, const uint8_t count) {
  datagram_t reply = nmranet_datagram_buffer_get(DATAGRAM_CONFIGURATION,
                                                 count + 5, OS_WAIT_FOREVER);
  nmranet_datagram_buffer_fill(reply, DATAGRAM_CONFIGURATION, command,
                               CONFIG_COMMAND, 1);
  nmranet_datagram_buffer_fill(reply, DATAGRAM_CONFIGURATION, address,
                               CONFIG_ADDRESS, sizeof(uint32_t));
  nmranet_datagram_buffer_fill(reply, DATAGRAM_CONFIGURATION, data, CONFIG_DATA,
                               count);
  nmranet_datagram_buffer_produce(node, from, reply, OS_WAIT_FOREVER);
}

/** Process the process a memory configuration datagram.
 * @param node node the datagram is to
 * @param datagram datagram to process, this pointer is stale upon return
 */
void datagram_memory_config(node_t node, datagram_t datagram) {
  Datagram *d = datagram;
  uint8_t *data = nmranet_datagram_payload(datagram);
  uint8_t space;
  uint32_t address;
  uint8_t space_offset = 0;
  uint8_t space_special = data[CONFIG_COMMAND] & CONFIG_COMMAND_FLAG_MASK;

  /* figure out what memory space we are operating on */
  switch (space_special) {
    default:
      /* This is not a special memory space */
      space = data[CONFIG_SPACE];
      space_offset = 1;
      break;
    case CONFIG_COMMAND_CDI:        /* fall through */
    case CONFIG_COMMAND_ALL_MEMORY: /* fall through */
    case CONFIG_COMMAND_CONFIG:
      space = CONFIG_SPACE_SPECIAL + space_special;
      break;
  }

  /* figure out the address */
  address = ((uint32_t)data[CONFIG_ADDRESS + 0] << 24) +
            ((uint32_t)data[CONFIG_ADDRESS + 1] << 16) +
            ((uint32_t)data[CONFIG_ADDRESS + 2] << 8) +
            ((uint32_t)data[CONFIG_ADDRESS + 3] << 0);

  /* figure out the command */
  switch (data[CONFIG_COMMAND] & CONFIG_COMMAND_MASK) {
    default:
      /* We don't know this command */
      break;
    case CONFIG_COMMAND_READ: {
      uint8_t count = data[CONFIG_COUNT + space_offset];
      assert(count <= 64);
      uint8_t command = CONFIG_COMMAND_READ_REPLY;
      if (space == CONFIG_SPACE_CDI) {
        command |= CONFIG_COMMAND_CDI;

        /* adjust count for size of CDI space */
        if (address > cdiSize) {
          count = 0;
        } else {
          count = (address + count) < cdiSize ? count : (cdiSize - address);
        }

        memory_config_reply(node, d->from, &command, &data[CONFIG_ADDRESS],
                            cdi + address, count);
      } else if (space == CONFIG_SPACE_ALL_MEMORY) {
        void *start = nmranet_memory_config_all(node, address, &count);
        command |= CONFIG_COMMAND_ALL_MEMORY;
        memory_config_reply(node, d->from, &command, &data[CONFIG_ADDRESS],
                            start, count);
      } else if (space == CONFIG_SPACE_CONFIG) {
        command |= CONFIG_COMMAND_ALL_MEMORY;
        memory_config_reply(node, d->from, &command, &data[CONFIG_ADDRESS],
                            (char *)0 + address, count);
      }
      break;
    }
    case CONFIG_COMMAND_OPTIONS: {
      uint16_t available = CONFIG_AVAIL_UR;
      uint8_t reply[6];
      reply[CONFIG_COMMAND] = CONFIG_COMMAND_OPTIONS_REPLY;
      reply[CONFIG_AVAILABLE] = available >> 8;
      reply[CONFIG_AVAILABLE + 1] = available & 0xFF;
      reply[CONFIG_LENGTHS] = 0;
      reply[CONFIG_HIGHEST] = CONFIG_SPACE_CDI;
      reply[CONFIG_LOWEST] = CONFIG_SPACE_CONFIG;
      nmranet_datagram_produce(node, d->from, DATAGRAM_CONFIGURATION, reply, 6,
                               OS_WAIT_FOREVER);
      break;
    }
    case CONFIG_COMMAND_INFORMATION: {
      uint32_t address_highest = sizeof(cdi);
      uint8_t reply[7];
      reply[CONFIG_COMMAND] =
          CONFIG_COMMAND_INFORMATION_REPLY | CONFIG_COMMAND_PRESENT;
      reply[CONFIG_ADDRESS_SPACE] = data[CONFIG_ADDRESS_SPACE];
      reply[CONFIG_ADDRESS_HIGHEST + 0] = (address_highest >> 24) & 0xFF;
      reply[CONFIG_ADDRESS_HIGHEST + 1] = (address_highest >> 16) & 0xFF;
      reply[CONFIG_ADDRESS_HIGHEST + 2] = (address_highest >> 8) & 0xFF;
      reply[CONFIG_ADDRESS_HIGHEST + 3] = (address_highest >> 0) & 0xFF;
      reply[CONFIG_FLAGS] = CONFIG_FLAG_RO;
      nmranet_datagram_produce(node, d->from, DATAGRAM_CONFIGURATION, reply, 7,
                               OS_WAIT_FOREVER);
      break;
    }
  }
}
