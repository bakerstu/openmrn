/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file EventCompatibility.hxx
 *
 * Compatibility layer for the C-style event handler interface.
 *
 * @author Balazs Racz
 * @date 3 Nov 2013
 */

#include "nmranet/EventCompatibility.hxx"

#include "nmranet_config.h"
#include "core/nmranet_event.h"
#include "core/nmranet_node_private.h"
#include "if/nmranet_if.h"

#ifdef CPP_EVENT_HANDLER


EventCompatibilityLayer::EventCompatibilityLayer() {

}

EventCompatibilityLayer::~EventCompatibilityLayer() {

}


void nmranet_event_consumer(node_t node, uint64_t event, unsigned int state) {
  
}

void nmranet_event_producer(node_t node, uint64_t event, unsigned int state) {
  
}

void nmranet_event_produce(node_t node, uint64_t event, unsigned int state) {
}

void nmranet_identify_consumers(node_t node, uint64_t event, uint64_t mask) {
  // Ignored: we'll do the global identify in IdentifyProducers.
}

void nmranet_identify_producers(node_t node, uint64_t event, uint64_t mask) {
  nmranet_event_packet_global(MTI_EVENTS_IDENTIFY_GLOBAL, {0, 0}, NULL);
}

// This is a trick we play on the linker to pull in these symbols. Otherwise we
// won't be able to link the binary, since there are back-references to these
// symbols from lower-level libraries.
void (*unused_h)(uint16_t, node_handle_t, node_t, const void*)=&nmranet_event_packet_addressed;

#endif
