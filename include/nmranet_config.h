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
 * \file nmranet_config.h
 * This file defines configuration options specific to NMRAnet.  These options
 * are typcially applied in the same file as @ref appl_main (main.c/main.cxx).
 * For example:
 *
 * @code
 * #include "nmranet_config.h"
 *
 * const char *nmranet_manufacturer = "Stuart W. Baker";
 * const char *nmranet_hardware_rev = "N/A";
 * const char *nmranet_software_rev = "0.1";
 * const size_t ALIAS_POOL_SIZE = 2;
 * const size_t DOWNSTREAM_ALIAS_CACHE_SIZE = 2;
 * const size_t UPSTREAM_ALIAS_CACHE_SIZE = 2;
 * @endcode
 *
 * @author Stuart W. Baker
 * @date 9 February 2013
 */

#ifndef _nmranet_config_h_
#define _nmranet_config_h_

#include <stdlib.h>

/** Define this macro to use the new C++ event transport implementation. */
#define CPP_EVENT_HANDLER

#ifdef __cplusplus
extern "C" {
#endif

/** Manufacture of the product. */
extern const char *nmranet_manufacturer;

/** Hardware revision of the product. */
extern const char *nmranet_hardware_rev;

/** Software revision of the product. */
extern const char *nmranet_software_rev;

/** Number of aliases to pool for instant use.
 */
extern const size_t ALIAS_POOL_SIZE;

/** Number of alias to node id mappings to cache for downstream nodes.
 */
extern const size_t DOWNSTREAM_ALIAS_CACHE_SIZE;

/** Number of alias to node id mappings to cache for upstream nodes.
 */
extern const size_t UPSTREAM_ALIAS_CACHE_SIZE;

/** Maximum number of datagram buffers that the stack can use.  A value of 0
 * means there is no limit */
extern const size_t DATAGRAM_POOL_SIZE;

/** Defines the size of stack for the CAN IF read thread. */
extern const size_t CAN_IF_READ_THREAD_STACK_SIZE;

/** Defines the size of stack for the datagram processing thread. */ 
extern const size_t DATAGRAM_THREAD_STACK_SIZE;

#ifdef __cplusplus
}
#endif

#endif /* _nmranet_config_h_ */
