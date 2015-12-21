/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file SimpleStack.cxx
 *
 * A complete OpenLCB stack for use in straightforward OpenLCB nodes.
 *
 * @author Balazs Racz
 * @date 18 Mar 2015
 */

#if defined(__linux__) || defined(__MACH__)
#include <termios.h> /* tc* functions */
#endif

#include "nmranet/SimpleStack.hxx"
#include "nmranet/SimpleNodeInfo.hxx"

namespace nmranet
{

SimpleCanStack::SimpleCanStack(const nmranet::NodeID node_id)
    : node_(&ifCan_, node_id)
{
    AddAliasAllocator(node_id, &ifCan_);
}

void SimpleCanStack::start_stack()
{
    // Opens the eeprom file and sends configuration update commands to all
    // listeners.
    configUpdateFlow_.open_file(CONFIG_FILENAME);
    configUpdateFlow_.init_flow();

    // Bootstraps the alias allocation process.
    ifCan_.alias_allocator()->send(ifCan_.alias_allocator()->alloc());

    // Adds memory spaces.
    if (config_enable_all_memory_space() == CONSTANT_TRUE)
    {
        auto *space = new ReadOnlyMemoryBlock(nullptr, 0xFFFFFFFFUL);
        memoryConfigHandler_.registry()->insert(
            nullptr, MemoryConfigDefs::SPACE_ALL_MEMORY, space);
        additionalComponents_.emplace_back(space);
    }
    {
        auto *space = new ReadOnlyMemoryBlock(
            reinterpret_cast<const uint8_t *>(&SNIP_STATIC_DATA),
            sizeof(SNIP_STATIC_DATA));
        memoryConfigHandler_.registry()->insert(
            &node_, MemoryConfigDefs::SPACE_ACDI_SYS, space);
        additionalComponents_.emplace_back(space);
    }
    {
        auto *space = new FileMemorySpace(
            SNIP_DYNAMIC_FILENAME, sizeof(SimpleNodeDynamicValues));
        memoryConfigHandler_.registry()->insert(
            &node_, MemoryConfigDefs::SPACE_ACDI_USR, space);
        additionalComponents_.emplace_back(space);
    }
    {
        auto *space = new ReadOnlyMemoryBlock(
            reinterpret_cast<const uint8_t *>(&CDI_DATA), strlen(CDI_DATA) + 1);
        memoryConfigHandler_.registry()->insert(
            &node_, MemoryConfigDefs::SPACE_CDI, space);
        additionalComponents_.emplace_back(space);
    }
    if (CONFIG_FILENAME != nullptr)
    {
        auto *space = new FileMemorySpace(CONFIG_FILENAME, CONFIG_FILE_SIZE);
        memory_config_handler()->registry()->insert(
            &node_, nmranet::MemoryConfigDefs::SPACE_CONFIG, space);
        additionalComponents_.emplace_back(space);
    }
}

void SimpleCanStack::add_gridconnect_port(const char *path, Notifiable *on_exit)
{
    int fd = ::open(path, O_RDWR);
    HASSERT(fd >= 0);
    LOG(INFO, "Adding device %s as fd %d", path, fd);
    create_gc_port_for_can_hub(&canHub0_, fd, on_exit);
}

#if defined(__linux__) || defined(__MACH__)
void SimpleCanStack::add_gridconnect_tty(
    const char *device, Notifiable *on_exit)
{
    int fd = ::open(device, O_RDWR);
    HASSERT(fd >= 0);
    LOG(INFO, "Adding device %s as fd %d", device, fd);
    create_gc_port_for_can_hub(&canHub0_, fd, on_exit);

    HASSERT(!tcflush(fd, TCIOFLUSH));
    struct termios settings;
    HASSERT(!tcgetattr(fd, &settings));
    cfmakeraw(&settings);
    cfsetspeed(&settings, B115200);
    HASSERT(!tcsetattr(fd, TCSANOW, &settings));
}
#endif
extern Pool *const __attribute__((__weak__)) g_incoming_datagram_allocator =
    init_main_buffer_pool();

} // namespace nmranet
