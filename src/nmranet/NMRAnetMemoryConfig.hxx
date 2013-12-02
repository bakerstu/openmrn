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
 * \file NMRAnetMemoryConfig.hxx
 * This file defines NMRAnet Memory Configuration Protocol(s).
 *
 * @author Stuart W. Baker
 * @date 13 October 2013
 */

#ifndef _NMRAnetMemoryhxx_
#define _NMRAnetMemoryhxx_

#include "utils/BufferQueue.hxx"
#include "nmranet/NMRAnetIf.hxx"

namespace NMRAnet {

class Node;

class MemoryConfig {
 public:
  /** Field offsets within datagram
   */
  enum offsets {
    COMMAND = 0,
    ADDRESS = 1,
    AVAILABLE = 1,
    ADDRESS_SPACE = 1,
    ADDRESS_HIGHEST = 2,
    LENGTHS = 3,
    HIGHEST = 4,
    LOWEST = 5,
    SPACE = 5,
    COUNT = 5,
    DATA = 5,
    FLAGS = 6,
    ADDRESS_LOWEST = 7,
    DESCRIPTION = 11,
    COUNT_MASK = 0x7F /**< the upper bit of the count is reserved */
  };

  /** Possible Commands for a configuration datagram.
   */
  enum commands {
    COMMAND_MASK = 0xFC,
    COMMAND_FLAG_MASK = 0x03,    /**< mask for special memory space flags */
    COMMAND_PRESENT_MASK = 0x01, /**< mask for address space present bit */
    COMMAND_WRITE = 0x00,        /**< command to write data to address space */
    COMMAND_WRITE_UNDER_MASK = 0x08, /**< command to write data under mask */
    COMMAND_WRITE_REPLY = 0x10,  /**< reply to write data to address space */
    COMMAND_WRITE_FAILED = 0x18, /**< failed to write data to address space */
    COMMAND_WRITE_STREAM = 0x20, /**< command to write data using a stream */
    COMMAND_READ = 0x40,         /**< command to read data from address space */
    COMMAND_READ_REPLY = 0x50,   /**< reply to read data from address space */
    COMMAND_READ_FAILED = 0x58,  /**< failed to read data from address space */
    COMMAND_READ_STREAM = 0x60,  /**< command to read data using a stream */
    COMMAND_OPTIONS = 0x80,
    COMMAND_OPTIONS_REPLY = 0x82,
    COMMAND_INFORMATION = 0x84,
    COMMAND_INFORMATION_REPLY = 0x86,
    COMMAND_LOCK = 0x88,            /**< lock the configuration space */
    COMMAND_LOCK_REPLY = 0x8A,      /**< unlock the configuration space */
    COMMAND_UNIQUE_ID = 0x8C,       /**< ask for a node unique id */
    COMMAND_UNIQUE_ID_REPLY = 0x8D, /**< node unique id */
    COMMAND_UPDATE_COMPLETE =
        0xA8, /**< indicate that a sequence of commands is complete */
    COMMAND_RESET = 0xA9,         /**< reset node to its power on state */
    COMMAND_FACTORY_RESET = 0xAA, /**< reset node to factory defaults */
    COMMAND_FREEZE = 0xA1,        /**< freeze operation of node */
    COMMAND_UNFREEZE = 0xA0,      /**< unfreeze operation of node */
    COMMAND_PRESENT = 0x01,       /**< address space is present */
    COMMAND_CDI = 0x03,           /**< flags for a CDI space */
    COMMAND_ALL_MEMORY = 0x02,    /**< flags for an all memory space */
    COMMAND_CONFIG = 0x01,        /**< flags for a config memory space */
  };

  /** Possible memory spaces.
   */
  enum spaces {
    SPACE_SPECIAL = 0xFC,    /**< offset for the special memory spaces */
    SPACE_CDI = 0xFF,        /**< CDI space */
    SPACE_ALL_MEMORY = 0xFE, /**< all memory space */
    SPACE_CONFIG = 0xFD,     /**< config memory space */
  };

  /** Possible available options.
   */
  enum available {
    AVAIL_WUM = 0x8000,   /**< write under mask supported */
    AVAIL_UR = 0x4000,    /**< unaligned reads supported */
    AVAIL_UW = 0x2000,    /**< unaligned writes supported */
    AVAIL_R0xFC = 0x0800, /**< read from adddress space 0xFC available */
    AVAIL_R0xFB = 0x0400, /**< read from adddress space 0xFB available */
    AVAIL_W0xFB = 0x0200, /**< write from adddress space 0xFB available */
  };

  /** Possible supported write lengths.
   */
  enum lengths {
    LENGTH_1 = 0x80,         /**< write length of 1 supported */
    LENGTH_2 = 0x40,         /**< write length of 2 supported */
    LENGTH_4 = 0x20,         /**< write length of 4 supported */
    LENGTH_63 = 0x10,        /**< write length of 64 supported */
    LENGTH_ARBITRARY = 0x02, /**< arbitrary write of any length supported */
    LENGTH_STREAM = 0x01,    /**< stream writes supported */
  };

  /** Possible address space information flags.
   */
  enum flags {
    FLAG_RO = 0x01,   /**< space is read only */
    FLAG_NZLA = 0x02, /**< space has a nonzero low address */
  };

 protected:
  /** Constructor.
   * @param cdi Configuration Data, else NULL to use default CDI
   */
  MemoryConfig(uint8_t *cdi);

  /** Default Destructor
   */
  ~MemoryConfig() {}

  /** Process a memory configuration datagram.
   * @param datagram datagram to process, this pointer is stale upon return
   */
  void process(Buffer *buffer);

 private:
  /** Helper function for replyoing to a memory config request.
   * @param from node we are replying to
   * @param command configuration command
   * @param address address within address space the data coresponds to
   * @param data location in memory to copy address space data from
   * @param count number of bytes to send
   */
  void reply(NodeHandle from, const void *command, const void *address,
             const void *data, const uint8_t count);

  /** Default hander for memory config all.
   * @param address offset in bytes to grab
   * @param count number of bytes to gram [in], number of bytes available [out]
   * @return pointer in memory to address space + offset
   */
  static void *all_memory(uint32_t address, uint8_t *count);

  /** Global CDI that is used if not overridden at the Node level */
  static const uint8_t globalCdi[];

  /** Global CDI size that is used if not overridden at the Node level */
  static size_t globalCdiSize;

  /** Node specific CDI */
  const uint8_t *cdi;

  /** size of Node specific CDI string */
  size_t cdiSize;

  /** Default Constructor.
   */
  MemoryConfig();

  DISALLOW_COPY_AND_ASSIGN(MemoryConfig);
};
};

#endif /* _NMRAnetMemoryhxx_ */
