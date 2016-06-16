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
#include <net/if.h>
#include <termios.h> /* tc* functions */
#endif
#if defined(__linux__)
#include <linux/sockios.h>
#endif

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "nmranet/SimpleStack.hxx"

#include "nmranet/EventHandler.hxx"
#include "nmranet/SimpleNodeInfo.hxx"

namespace nmranet
{

SimpleCanStackBase::SimpleCanStackBase(const nmranet::NodeID node_id)
{
    AddAliasAllocator(node_id, &ifCan_);
}

SimpleCanStack::SimpleCanStack(const nmranet::NodeID node_id)
    : SimpleCanStackBase(node_id)
    , node_(&ifCan_, node_id)
{
}

void SimpleCanStackBase::start_stack()
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

    // Calls node-specific startup hook.
    start_node();
}

void SimpleCanStack::start_node()
{
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

void SimpleCanStackBase::restart_stack()
{
    node()->clear_initialized();
    ifCan_.alias_allocator()->reinit_seed();
    ifCan_.local_aliases()->clear();
    ifCan_.remote_aliases()->clear();
    // Deletes all reserved aliases from the queue.
    while (!ifCan_.alias_allocator()->reserved_aliases()->empty())
    {
        Buffer<AliasInfo> *a = static_cast<Buffer<AliasInfo> *>(
            ifCan_.alias_allocator()->reserved_aliases()->next().item);
        if (a)
        {
            a->unref();
        }
    }

    // Bootstraps the fresh alias allocation process.
    ifCan_.alias_allocator()->send(ifCan_.alias_allocator()->alloc());
    extern void StartInitializationFlow(Node * node);
    StartInitializationFlow(node());
}

void SimpleCanStackBase::create_config_file_if_needed(
    const InternalConfigData &cfg, uint16_t expected_version,
    unsigned file_size)
{
    HASSERT(CONFIG_FILENAME);
    struct stat statbuf;
    bool reset = false;
    int st = ::stat(CONFIG_FILENAME, &statbuf);
    if (st < 0 && (errno == ENOENT))
    {
        // Create file.
        LOG(INFO, "Creating config file %s", CONFIG_FILENAME);
        if (creat(CONFIG_FILENAME, S_IRUSR | S_IWUSR) < 0)
        {
            LOG(LEVEL_ERROR, "Failed to create config file: %s",
                strerror(errno));
        }
        HASSERT(stat(CONFIG_FILENAME, &statbuf) == 0);
        reset = true;
    }
    int fd = configUpdateFlow_.open_file(CONFIG_FILENAME);
    if (statbuf.st_size < (ssize_t)file_size)
        reset = true;
    if (cfg.version().read(fd) != expected_version)
        reset = true;
    if (!reset)
        return;

    // Clears the file.
    lseek(fd, 0, SEEK_SET);
    static const unsigned bufsize = 128;
    char *buf = (char *)malloc(bufsize);
    HASSERT(buf);
    memset(buf, 0xff, bufsize);
    unsigned len = file_size;
    while (len > 0)
    {
        ssize_t c = write(fd, buf, std::min(len, bufsize));
        HASSERT(c >= 0);
        len -= c;
    }
    free(buf);

    // Initializes basic structures in the file.
    cfg.version().write(fd, expected_version);
    cfg.next_event().write(fd, 0);
    // ACDI version byte. This is not very nice because we cannot be
    // certain that the EEPROM starts with the ACDI data. We'll check it
    // though.
    HASSERT(SNIP_DYNAMIC_FILENAME == CONFIG_FILENAME);
    Uint8ConfigEntry(0).write(fd, 2);
    factory_reset_all_events(cfg, fd);
    configUpdateFlow_.factory_reset();
}

void SimpleCanStackBase::check_version_and_factory_reset(
    const InternalConfigData &cfg, uint16_t expected_version, bool force)
{
    HASSERT(CONFIG_FILENAME);
    int fd = configUpdateFlow_.open_file(CONFIG_FILENAME);
    if (cfg.version().read(fd) != expected_version)
    {
        /// @todo (balazs.racz): We need to clear the eeprom. Best would be if
        /// there was an ioctl to return the eeprom to factory default state by
        /// just erasing the segments.
        cfg.version().write(fd, expected_version);
        cfg.next_event().write(fd, 0);
        // ACDI version byte. This is not very nice because we cannot be
        // certain that the EEPROM starts with the ACDI data. We'll check it
        // though.
        HASSERT(SNIP_DYNAMIC_FILENAME == CONFIG_FILENAME);
        Uint8ConfigEntry(0).write(fd, 2);
        force = true;
    }
    if (force)
    {
        factory_reset_all_events(cfg, fd);
        configUpdateFlow_.factory_reset();
    }
}

extern const uint16_t CDI_EVENT_OFFSETS[];

void SimpleCanStackBase::factory_reset_all_events(
    const InternalConfigData &cfg, int fd)
{
    // First we find the event count.
    uint16_t new_next_event = cfg.next_event().read(fd);
    uint16_t next_event = new_next_event;
    for (unsigned i = 0; CDI_EVENT_OFFSETS[i]; ++i)
    {
        ++new_next_event;
    }
    // We block off the event IDs first.
    cfg.next_event().write(fd, new_next_event);
    // Then we write them to eeprom.
    for (unsigned i = 0; CDI_EVENT_OFFSETS[i]; ++i)
    {
        EventId id = node()->node_id();
        id <<= 16;
        id |= next_event++;
        EventConfigEntry(CDI_EVENT_OFFSETS[i]).write(fd, id);
    }
}

void SimpleCanStackBase::add_gridconnect_port(
    const char *path, Notifiable *on_exit)
{
    int fd = ::open(path, O_RDWR);
    HASSERT(fd >= 0);
    LOG(INFO, "Adding device %s as fd %d", path, fd);
    create_gc_port_for_can_hub(&canHub0_, fd, on_exit);
}

#if defined(__linux__) || defined(__MACH__)
void SimpleCanStackBase::add_gridconnect_tty(
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
#if defined(__linux__)
void SimpleCanStackBase::add_socketcan_port_select(
    const char *device, int loopback)
{
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    // Set the blocking limit to the minimum allowed, typically 1024 in Linux
    int sndbuf = 0;
    setsockopt(s, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    // turn on/off loopback
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    // setup error notifications
    can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_LOSTARB |
        CAN_ERR_CRTL | CAN_ERR_PROT | CAN_ERR_TRX | CAN_ERR_ACK |
        CAN_ERR_BUSOFF | CAN_ERR_BUSERROR | CAN_ERR_RESTARTED;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
    strcpy(ifr.ifr_name, device);

    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    bind(s, (struct sockaddr *)&addr, sizeof(addr));

    auto *port = new HubDeviceSelect<CanHubFlow>(&canHub0_, s);
    additionalComponents_.emplace_back(port);
}
#endif
extern Pool *const __attribute__((__weak__)) g_incoming_datagram_allocator =
    init_main_buffer_pool();

} // namespace nmranet
