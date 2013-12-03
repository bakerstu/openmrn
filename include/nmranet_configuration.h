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
 * \file nmranet_configuration.h
 * This file defines specifics about the configuration protocol.
 *
 * @author Stuart W. Baker
 * @date 16 March 2013
 */

#ifndef _nmranet_configuration_h_
#define _nmranet_configuration_h_

#ifdef __cplusplus
extern "C" {
#endif

/** Field offsets within datagram
 */
enum offsets {
    CONFIG_COMMAND = 0,
    CONFIG_ADDRESS = 1,
    CONFIG_AVAILABLE = 1,
    CONFIG_ADDRESS_SPACE = 1,
    CONFIG_ADDRESS_HIGHEST = 2,
    CONFIG_LENGTHS = 3,
    CONFIG_HIGHEST = 4,
    CONFIG_LOWEST = 5,
    CONFIG_SPACE = 5,
    CONFIG_COUNT = 5,
    CONFIG_DATA = 5,
    CONFIG_FLAGS = 6,
    CONFIG_ADDRESS_LOWEST = 7,
    CONFIG_DESCRIPTION = 11,
};

/** Possible Commands for a configuration datagram.
 */
enum commands {
    CONFIG_COMMAND_MASK = 0xFC,
    CONFIG_COMMAND_FLAG_MASK = 0x03, /**< mask for special memory space flags */
    CONFIG_COMMAND_PRESENT_MASK
    = 0x01, /**< mask for address space present bit */
    CONFIG_COMMAND_READ = 0x40,
    CONFIG_COMMAND_READ_REPLY = 0x50,
    CONFIG_COMMAND_OPTIONS = 0x80,
    CONFIG_COMMAND_OPTIONS_REPLY = 0x82,
    CONFIG_COMMAND_INFORMATION = 0x84,
    CONFIG_COMMAND_INFORMATION_REPLY = 0x86,
    CONFIG_COMMAND_PRESENT = 0x01,    /**< address space is present */
    CONFIG_COMMAND_CDI = 0x03,        /**< flags for a CDI space */
    CONFIG_COMMAND_ALL_MEMORY = 0x02, /**< flags for an all memory space */
    CONFIG_COMMAND_CONFIG = 0x01,     /**< flags for a config memory space */
};

/** Possible memory spaces.
 */
enum spaces {
    CONFIG_SPACE_SPECIAL = 0xFC,    /**< offset for the special memory spaces */
    CONFIG_SPACE_CDI = 0xFF,        /**< CDI space */
    CONFIG_SPACE_ALL_MEMORY = 0xFE, /**< all memory space */
    CONFIG_SPACE_CONFIG = 0xFD,     /**< config memory space */
};

/** Possible available options.
 */
enum available {
    CONFIG_AVAIL_WUM = 0x8000,   /**< write under mask supported */
    CONFIG_AVAIL_UR = 0x4000,    /**< unaligned reads supported */
    CONFIG_AVAIL_UW = 0x2000,    /**< unaligned writes supported */
    CONFIG_AVAIL_R0xFC = 0x0800, /**< read from adddress space 0xFC available */
    CONFIG_AVAIL_R0xFB = 0x0800, /**< read from adddress space 0xFC available */
    CONFIG_AVAIL_W0xFB
    = 0x0800, /**< write from adddress space 0xFC available */
};

/** Possible supported write lengths.
 */
enum lengths {
    CONFIG_LENGTH_1 = 0x80,  /**< write length of 1 supported */
    CONFIG_LENGTH_2 = 0x40,  /**< write length of 2 supported */
    CONFIG_LENGTH_4 = 0x20,  /**< write length of 4 supported */
    CONFIG_LENGTH_63 = 0x10, /**< write length of 64 supported */
    CONFIG_LENGTH_ARBITRARY
    = 0x02,                      /**< arbitrary write of any length supported */
    CONFIG_LENGTH_STREAM = 0x01, /**< stream writes supported */
};

/** Possible address space information flags.
 */
enum flags {
    CONFIG_FLAG_RO = 0x01,   /**< space is read only */
    CONFIG_FLAG_NZLA = 0x02, /**< space has a nonzero low address */
};

#ifdef __cplusplus
}
#endif

#endif /* _nmranet_configuration_h_ */
