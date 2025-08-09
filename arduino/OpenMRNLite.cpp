/** \copyright
 * Copyright (c) 2018, Balazs Racz
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
 * \file OpenMRN.cpp
 * 
 * Implementation that needs to be compiled for the Arduino.
 *
 * @author Balazs Racz
 * @date 24 July 2018
 */

#include <OpenMRNLite.h>

OVERRIDE_CONST(gridconnect_bridge_max_incoming_packets, 5);

namespace openmrn_arduino {

OpenMRN::OpenMRN(openlcb::NodeID node_id)
{
    init(node_id);
}

#ifdef ESP_PLATFORM
extern "C" {

#ifndef OPENMRN_EXCLUDE_REBOOT_IMPL
/// Reboots the ESP32 via the arduino-esp32 provided restart function.
void reboot()
{
    ESP.restart();
}
#endif // OPENMRN_EXCLUDE_REBOOT_IMPL

#ifndef OPENMRN_EXCLUDE_FREE_HEAP_IMPL
ssize_t os_get_free_heap()
{
    return ESP.getFreeHeap();
}
#endif // OPENMRN_EXCLUDE_FREE_HEAP_IMPL

}
#endif // ESP_PLATFORM

} // namespace openmrn_arduino

#ifdef ESP_PLATFORM

#include "lwip/ip_addr.h"
#include "lwip/err.h"
#include "lwip/netif.h"

/*
temporary hack due to:

/home/runner/.cache/arduino/cores/esp32_esp32_esp32c3_e60f223ab547c7c01b5bb8c151783c5f/core.a @/home/runner/.arduino15/packages/esp32/tools/esp32-arduino-libs/idf-release_v5.3-cfea4f7c-v1/esp32c3/flags/ld_libs -Wl,--end-group -Wl,-EL -o /home/runner/.cache/arduino/sketches/1429F777C89502EA84E469B35328AFC7/ESP32C3IOBoard.ino.elf
/home/runner/.arduino15/packages/esp32/tools/esp-rv32/2405/bin/../lib/gcc/riscv32-esp-elf/13.2.0/../../../../riscv32-esp-elf/bin/ld: /home/runner/.arduino15/packages/esp32/tools/esp32-arduino-libs/idf-release_v5.3-cfea4f7c-v1/esp32c3/lib/liblwip.a(ip6.c.obj): in function `ip6_input':
/home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/lwip/lwip/src/core/ipv6/ip6.c:1127:(.text.ip6_input+0xa2): undefined reference to `lwip_hook_ip6_input'
collect2: error: ld returned 1 exit status

tracked as https://github.com/espressif/arduino-esp32/issues/10084
below snippet was copied from the issue and must be placed in global namespace
*/
extern "C" int lwip_hook_ip6_input(struct pbuf *p, struct netif *inp) __attribute__((weak));
extern "C" int lwip_hook_ip6_input(struct pbuf *p, struct netif *inp)
{
  if (ip6_addr_isany_val(inp->ip6_addr[0].u_addr.ip6))
  {
    // We don't have an LL address -> eat this packet here, so it won't get accepted on input netif
    pbuf_free(p);
    return 1;
  }
  return 0;
}
#endif
