/** @copyright
 * Copyright (c) 2024, Balazs Racz, 2025, Stuart Baker
 * All rights reserved
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
 * @file EspIdfWiFi.cxx
 *
 * Interface for interacting with WiFi services.
 *
 * @author Balazs Racz, extended by Stuart Baker
 * @date 27 Aug 2024, extended starting Jun 17 2025
 */

#include "freertos_drivers/esp_idf/EspIdfWiFi.hxx"

#include <ifaddrs.h>

#include <esp_event.h>
#include <esp_wifi.h>
#include <lwip/inet.h>
#include <lwip/netdb.h>
#include <mdns.h>

#include "utils/format_utils.hxx"

/// Scanning parameter configuration.
static const wifi_scan_config_t SCAN_CONFIG = 
{
    .ssid = nullptr,
    .bssid = nullptr,
    .channel = 0,
    .show_hidden = true,
    .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    .scan_time = {.active = {.min = 0, .max = 120}, .passive = 360},
    .home_chan_dwell_time = 30,
};

//
// mdns_publish()
//
void mdns_publish(const char *name, const char *service, uint16_t port)
{
    static_cast<EspIdfWiFi*>(WiFiInterface::instance())
        ->mdns_service_add(service, port);
}

//
// mdns_unpublish()
//
void mdns_unpublish(const char *name, const char *service)
{
    static_cast<EspIdfWiFi*>(WiFiInterface::instance())
        ->mdns_service_remove(service);
}

//
// mdns_scan()
//
void mdns_scan(const char *service)
{
    static_cast<EspIdfWiFi*>(WiFiInterface::instance())
        ->mdns_scan(service);
}

//
// mdns_lookup()
//
int mdns_lookup(const char *service, struct addrinfo *hints,
                struct addrinfo **addr)
{
    return static_cast<EspIdfWiFi*>(WiFiInterface::instance())
        ->mdns_lookup(service, hints, addr);
}

/// Helper method for getifaddrs().
/// @param iface interface instance pointer
/// @param name interface name
static struct ifaddrs *getifaddrs_helper(esp_netif_t *iface, const char *name)
{
#if defined(ESP_IDF_WIFI_IPV6)
#error "Need to add IPV6 support for getifaddrs() implementation."
#endif
    if (iface == nullptr)
    {
        return nullptr;
    }

    struct ifaddrs *if_addrs =
         (struct ifaddrs*)calloc(1, sizeof(struct ifaddrs));
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(iface, &ip_info) == ESP_OK)
    {
        struct sockaddr_in *addr_in =
            (struct sockaddr_in*)calloc(2, sizeof(struct sockaddr_in));
        addr_in[0].sin_family = AF_INET;
        addr_in[0].sin_addr.s_addr = ip_info.ip.addr;
        addr_in[1].sin_family = AF_INET;
        addr_in[1].sin_addr.s_addr = ip_info.netmask.addr;

        LOG(INFO, "wifi: getifaddrs() ip: %s, netmask: %s",
            ipv4_to_string(addr_in[0].sin_addr.s_addr).c_str(),
            ipv4_to_string(addr_in[1].sin_addr.s_addr).c_str());

        if_addrs->ifa_addr = (struct sockaddr*)addr_in;
        if_addrs->ifa_netmask = (struct sockaddr*)(addr_in + 1);
    }
    if_addrs->ifa_name = (char*)calloc(1, 4);
    strncpy(if_addrs->ifa_name, name, 3);
    if_addrs->ifa_next = nullptr;
    return if_addrs;
}

/// Get interface addresses
/// @param ifap pointer to return the list of interface addresses to
/// @return 0 upon success, else -1 with errno set to indicate the error
int getifaddrs(struct ifaddrs **ifap)
{
    esp_netif_t *netif_ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    esp_netif_t *netif_sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    struct ifaddrs *if_addrs_ap = getifaddrs_helper(netif_ap, "ap");
    struct ifaddrs *if_addrs_sta = getifaddrs_helper(netif_sta, "sta");

    if (netif_ap)
    {
        *ifap = if_addrs_ap;
        if_addrs_ap->ifa_next = if_addrs_sta;
    }
    else
    {
        *ifap = if_addrs_sta;
    }
    if (*ifap == nullptr)
    {
        errno = ENODEV;
        return -1;
    }
    return 0;
}

/// Get the string version of a given error.
/// @return the string equivalent of the passed error code.
const char *gai_strerror(int __ecode)
{
    switch (__ecode)
    {
        default:
            return "gai_strerror unknown";
        case EAI_AGAIN:
            return "temporary failure";
        case EAI_FAIL:
            return "non-recoverable failure";
        case EAI_MEMORY:
            return "memory allocation failure";
    }
}

/// Free interface address list returned by getifaddrs()
/// @param ifa list of interface addresses to free
void freeifaddrs(struct ifaddrs *ifa)
{
    while (ifa)
    {
        struct ifaddrs* current = ifa;
        ifa = ifa->ifa_next;

        if (current->ifa_addr)
        {
            HASSERT(current->ifa_addr->sa_family == AF_INET);
            free(current->ifa_addr);
        }
        free(current->ifa_name);
        free(current);
    }
}

//
// EspIdfWiFi::stop()
//
void EspIdfWiFi::stop()
{
    esp_wifi_stop();
}

//
// EspIdfWiFi::connect()
//
WlanConnectResult EspIdfWiFi::connect(
    const char *ssid, const char *pass, SecurityType sec_type)
{
    securityFailure_ = false;
    sta_connect(
        std::string(ssid), std::string(pass), sec_type_translate(sec_type));
    return WlanConnectResult::CONNECT_OK;
}

//
// EspIdfWiFi::disconnect()
//
void EspIdfWiFi::disconnect()
{
    /// @todo Do we need to manually handle status flags and user callbacks, or
    ///       will the ESP events be delivered that can do this for us? Need to
    ///       test.
    esp_wifi_disconnect();
}

//
// EspIdfWiFi::setup_ap()
//
void EspIdfWiFi::setup_ap(
        const char *ssid, const char *pass, SecurityType sec_type)
{
    OSMutexLock locker(this);
    strncpy(userCfg_.ap_.ssid_, ssid, MAX_SSID_LEN);
    userCfg_.ap_.ssid_[sizeof(userCfg_.ap_.ssid_) - 1] = '\0';
    strncpy(userCfg_.ap_.pass_, pass, MAX_PASSPHRASE_LEN);
    userCfg_.ap_.pass_[sizeof(userCfg_.ap_.pass_) - 1] = '\0';
    userCfg_.ap_.sec_ = sec_type_translate(sec_type);
    config_sync();
}

//
// EspIdfWiFi::set_role()
//
void EspIdfWiFi::set_role(WlanRole new_role)
{
    // This is a single byte copy, it is already atomic.
    static_assert(sizeof(userCfg_.mode_)== sizeof(uint8_t));
    userCfg_.mode_ = new_role;
    config_sync();
}

//
// EspIdfWiFi::profile_add()
//
int EspIdfWiFi::profile_add(const char *ssid, const char *pass,
    SecurityType sec_type, uint8_t priority)
{
    // find an "empty" profie
    OSMutexLock locker(this);
    int index = find_sta_profile("");
    if (index >= 0)
    {
        strncpy(userCfg_.sta_[index].pass_, pass, MAX_PASS_SIZE);
        userCfg_.sta_[index].pass_[MAX_PASS_SIZE] = '\0';
        userCfg_.sta_[index].sec_ = sec_type_encode(sec_type);
        strncpy(userCfg_.sta_[index].ssid_, ssid, MAX_SSID_SIZE);
        userCfg_.sta_[index].ssid_[MAX_SSID_SIZE] = '\0';
        config_sync();
    }
    return index;
}

//
// EspIdfWiFi::profile_get()
//
int EspIdfWiFi::profile_get(
    int index, char ssid[], SecurityType *sec_type, uint8_t *priority)
{
    OSMutexLock locker(this);
    if (index >= ARRAYSIZE(userCfg_.sta_))
    {
        return -1;
    }
    strncpy(ssid, userCfg_.sta_[index].ssid_, MAX_SSID_SIZE);
    ssid[MAX_SSID_SIZE] = '\0';
    *sec_type = sec_type_encode(userCfg_.sta_[index].sec_);
    *priority = 0;
    return 0;
}

//
// EspIdfWiFi::scan()
//
void EspIdfWiFi::scan()
{
    esp_wifi_scan_start(&SCAN_CONFIG, false);
}

//
// EspIdfWiFi::rssi()
//
int EspIdfWiFi::rssi()
{
    if (connected_)
    {
        int rssi;
        esp_wifi_sta_get_rssi(&rssi);
        return rssi;
    }
    return 0;
}

//
// EspIdfWiFi::mdns_service_add()
//
void EspIdfWiFi::mdns_service_add(const char *service, uint16_t port)
{
    std::string name;
    std::string proto;
    mdns_split(service, &name, &proto);
    LOG(INFO, "wifi: MDNS service add, name: %s, proto: %s, port: %u",
        name.c_str(), proto.c_str(), port);
    OSMutexLock locker(this);
    if (!mdnsAdvInhibit_)
    {
        ::mdns_service_add(
            nullptr, name.c_str(), proto.c_str(), port, nullptr, 0);
    }
    mdnsServices_.emplace_back(std::move(name), std::move(proto), port);
}

//
// EspIdfWiFi::mdns_service_remove()
//
void EspIdfWiFi::mdns_service_remove(const char *service)
{
    std::string name;
    std::string proto;
    mdns_split(service, &name, &proto);
    LOG(INFO, "wifi: MDNS service remove, service: %s, proto: %s",
        name.c_str(), proto.c_str());
    OSMutexLock locker(this);
    if (!mdnsAdvInhibit_)
    {
        ::mdns_service_remove(name.c_str(), proto.c_str());
    }
    for (auto it = mdnsServices_.begin(); it != mdnsServices_.end(); ++it)
    {
        if ((*it).name_ == name && (*it).proto_ == proto)
        {
            mdnsServices_.erase(it);
            return;
        }
    }
}

//
// EspIdfWiFi::mdns_lookup()
//
int EspIdfWiFi::mdns_lookup(
    const char *service, struct addrinfo *hints, struct addrinfo **addr)
{
    bool looking = false;
    *addr = nullptr;
    OSMutexLock locker(this);
    for (auto it = mdnsClientCache_.begin(); it != mdnsClientCache_.end(); ++it)
    {
        if ((*it).service_ != service)
        {
            // Not a match.
            continue;
        }

        // Found a potential match. Check for an address list
        looking = true;
        size_t addr_cnt = (*it).addr_.size();
        if (addr_cnt == 0)
        {
            // No resolved addresses to report.
            continue;
        }

        constexpr size_t total_size =
            sizeof(struct addrinfo) + sizeof(struct sockaddr_storage);
        static_assert(
            total_size <= NETDB_ELEM_SIZE, "total_size > NETDB_ELEM_SIZE");
        struct addrinfo *current = *addr;
        struct addrinfo *last = nullptr;

        for (auto lit = (*it).addr_.begin(); lit != (*it).addr_.end(); ++lit)
        {
// The caller is expected to free the struct addrinfo using the method
// freeaddrinfo. In the lwIP implementation, they use a special buffer pool
// to free the struct addrinfo to. Therefore, we must also allocate from the
// same pool.
#if !defined(LWIP_HDR_MEMP_H)
#error "lwIP memory pool implementation required and not found."
#endif
            current = (struct addrinfo*)memp_malloc(MEMP_NETDB);
            if (!current)
            {
                // Out of memory in the pool
                break;
            }
            memset(current, 0, total_size);
            if (last)
            {
                // Link the last item to us.
                last->ai_next = current;
            }
            else
            {
                // This is the first item.
                *addr = current;
            }
            MDNSCacheAddr *ca = &(*lit);
#if defined(ESP_IDF_WIFI_IPV6)
            size_t addr_len = AF_INET == ca->family_ ?
                sizeof(struct sockaddr_in) : sizeof(struct sockaddr_in6);
            HASSERT(ca->family_ == AF_INET || ca->family_ == AF_INET6);
#else
            size_t addr_len = sizeof(struct sockaddr_in);
            HASSERT(ca->family_ == AF_INET);
#endif
            current->ai_family = ca->family_;
            current->ai_addrlen = sizeof(struct sockaddr_storage);
            current->ai_addr = (struct sockaddr*)(current + 1);
            memcpy(current->ai_addr, &ca->addr_, addr_len);
            last = current;
            LOG(INFO, "wifi: mdns_lookup() address: %s, port: %u",
                ipv4_to_string(ntohl(ca->addrIn_.sin_addr.s_addr)).c_str(),
                ntohs(ca->addrIn_.sin_port));
        }
        if (last)
        {
            last->ai_next = nullptr;
        }
        return *addr ? 0 : EAI_MEMORY;
    }
    if (!looking)
    {
        // Not currently looking for the service. Start looking.
        mdns_scan(service);
    }
    return EAI_AGAIN;
}

//
// EspIdfWiFi::mdns_scan()
//
void EspIdfWiFi::mdns_scan(const char *service)
{
    bool match = false;
    OSMutexLock locker(this);
    for (auto it = mdnsClientCache_.begin(); it != mdnsClientCache_.end(); ++it)
    {
        if ((*it).service_ == service)
        {
            match = true;
        }
    }
    if (!match)
    {
        // Start looking for a new service.
        mdnsClientCache_.emplace_back(std::move(std::string(service)));
    }
    if (!mdnsClientStarted_)
    {
        // Start the client if not already started.
        mdnsClientStarted_ = true;
        notify();
        LOG(INFO, "wifi: mdns_scan() start.");
    }
    else if (!mdnsClientTrigRefresh_)
    {
        // Trigger the mDNS state machine to execute early. If a query is
        // already taking place, a new one will start immediately following it.
        // If a query it not taking place, one will be started immediately.
        mdnsClientTrigRefresh_ = true;
        CallbackExecutable *e =
            new CallbackExecutable([this](){this->timer_.ensure_triggered();});
        this->service()->executor()->add(e);
        LOG(INFO, "wifi: mdns_scan() trigger.");
    }
}

//
// EspIdfWiFi::factory_default()
//
void EspIdfWiFi::factory_default()
{
    LOG(INFO, "wifi: factory_default()");
    memset(&userCfg_, 0, sizeof(userCfg_));
    userCfg_.magic_ = WIFI_CONFIG_INIT_MAGIC;
    userCfg_.mode_ = WlanRole::AP_STA;
    strncpy(userCfg_.ap_.ssid_, hostname_, MAX_SSID_SIZE);
    userCfg_.ap_.ssid_[MAX_SSID_SIZE] = '\0';
    strcpy(userCfg_.ap_.pass_, DEFAULT_PASSWORD);
    userCfg_.ap_.sec_ = 2;
    strcpy(userCfg_.sta_[0].ssid_, DEFAULT_STA_SSID);
    strcpy(userCfg_.sta_[0].pass_, DEFAULT_PASSWORD);
    userCfg_.sta_[0].sec_ = 2;
}

//
// EspIdfWiFi::MDNSCacheItem::reset()
void EspIdfWiFi::MDNSCacheItem::reset(void *handle)
{
    if (searchHandle_ && handle != searchHandle_)
    {
        mdns_query_async_delete((mdns_search_once_t*)searchHandle_);
    }
    searchHandle_ = handle;
}

//
// EspIdfWiFi::mdns_start()
//
StateFlowBase::Action EspIdfWiFi::mdns_start()
{
    HASSERT(mdnsClientCache_.size() > 0);
    if (mdnsAdvInhibitSta_)
    {
        mdns_adv_inhibit();
        mdns_restore_sta();
        mdnsAdvInhibitStaActive_ = true;
    }
    return call_immediately(STATE(mdns_query));
}

//
// EspIdfWiFi::mdns_query()
//
StateFlowBase::Action EspIdfWiFi::mdns_query()
{
    OSMutexLock lock(this);
    mdnsClientTrigRefresh_ = false;
    for (auto it = mdnsClientCache_.begin(); it != mdnsClientCache_.end(); ++it)
    {
        std::string name;
        std::string proto;
        mdns_split((*it).service_.c_str(), &name, &proto);
        (*it).reset((void*)mdns_query_async_new(
            nullptr, name.c_str(), proto.c_str(), MDNS_TYPE_PTR,
            MDNS_QUERY_ASYNC_NEW_TIMEOUT_MSEC, 3, nullptr));
        LOG(VERBOSE, "wifi: mDNS new search query: %s.%s",
            name.c_str(), proto.c_str());
    }

    return sleep_and_call(&timer_,
        MDNS_QUERY_CHECK_TIMEOUT_NSEC, STATE(mdns_check));
}

//
// EspIdfWiFi::mdns_check()
//
StateFlowBase::Action EspIdfWiFi::mdns_check()
{
    bool search_still_active = false;
    OSMutexLock lock(this);
    // Loop through all of the service clients.
    for (auto it = mdnsClientCache_.begin(); it != mdnsClientCache_.end(); ++it)
    {
        if ((*it).get() == nullptr)
        {
            // No active search for this cache entry.
            LOG(VERBOSE, "wifi: mDNS no active search for this cache entry.");
            continue;
        }
        mdns_result_t *mdns_result = nullptr;
        bool result = mdns_query_async_get_results(
            (mdns_search_once_t*)(*it).get(), 0, &mdns_result, nullptr);
        if (!result)
        {
            // Results not ready yet.
            LOG(VERBOSE, "wifi: mDNS client results not ready yet.");
            search_still_active = true;
            continue;
        }
        if (mdns_result == nullptr)
        {
            // No results for this service.
            LOG(VERBOSE, "wifi: No mDNS client results for this service.");
            (*it).reset();
            continue;
        }

        // Loop through all the results.
        for (mdns_result_t *cur_result = mdns_result;
            cur_result; cur_result = cur_result->next)
        {
            MDNSCacheAddr cache_addr;
            cache_addr.port_ = htons(cur_result->port);

            for (mdns_ip_addr_t *addr = cur_result->addr; addr;
                addr = addr->next)
            {
                // For some reason, the mdns_result_t::ip_protocol cannot
                // be trusted. The address results list can contain both
                // IPv4 and IPv6 addresses.
                if (addr->addr.type == ESP_IPADDR_TYPE_V4)
                {
                    cache_addr.family_ = AF_INET;
                }
#if defined(ESP_IDF_WIFI_IPV6)
                // OpenMRN does not support IPv6 for mdns_lookup() yet.
                else if (addr->addr.type == ESP_IPADDR_TYPE_V6)
                {
                    cache_addr.family_ = AF_INET6;
                }
#endif
                else
                {
                    // Unknown protocol.
                    continue;
                }
                bool duplicate = false;
                for (auto ait = (*it).addr_.begin();
                    !duplicate && ait != (*it).addr_.end(); ++ait)
                {
                    duplicate = (*ait).family_ == cache_addr.family_;
                    duplicate &= (*ait).port_ == cache_addr.port_;
                    if (cache_addr.family_ == AF_INET)
                    {
                        duplicate &= (*ait).addrIn_.sin_addr.s_addr ==
                            addr->addr.u_addr.ip4.addr;
                    }
#if defined(ESP_IDF_WIFI_IPV6)
                    else if (cache_addr.family_ == AF_INET6)
                    {
                        duplicate &= 
                            !memcmp((*ait).addrIn6_.sin6_addr.un.u32_addr,
                                addr->addr.u_addr.ip6.addr, 16);
                    }
#endif
                }
                if (!duplicate)
                {
                    if (cache_addr.family_ == AF_INET)
                    {
                        cache_addr.addrIn_.sin_addr.s_addr =
                            addr->addr.u_addr.ip4.addr;
                        cache_addr.addrIn_.sin_port = cache_addr.port_;
                        LOG(INFO, "wifi: mDNS service discovered: %s\n"
                            "     %s %s:%u", cur_result->hostname,
                            (*it).service_.c_str(),
                            ipv4_to_string(ntohl(
                                cache_addr.addrIn_.sin_addr.s_addr)).c_str(),
                            ntohs(cache_addr.port_));
                    }
#if defined(ESP_IDF_WIFI_IPV6)
                    else if (cache_addr.family_ == AF_INET6)
                    {
                        memcpy(cache_addr.addrIn6_.sin6_addr.un.u32_addr,
                            addr->addr.u_addr.ip6.addr, 16);
                        cache_addr.addrIn6_.sin6_port = cache_addr.port_;
                        LOG(INFO, "wifi: mDNS service discovered: %s\n"
                            "     %s %s:%u", cur_result->hostname,
                            (*it).service_.c_str(),
                            ipv6_to_string(
                                cache_addr.addrIn6_.sin6_addr.un.u8_addr)
                                    .c_str(),
                            ntohs(cache_addr.port_));
                    }
#endif
                    if ((*it).addr_.size() >= MDNS_RESULT_COUNT_MAX)
                    {
                        LOG(VERBOSE,
                            "wifi: mDNS service evict old cache entry");
                        (*it).addr_.pop_back();
                    }
                    (*it).addr_.emplace_front(cache_addr);
                }
                else
                {
                    LOG(VERBOSE, "wifi: mDNS service discovered duplicate.");
                }
            }
        }
        mdns_query_results_free(mdns_result);
        (*it).reset();
    }
    if (search_still_active)
    {
        // There are still active queries, sleep and check again later.
        return sleep_and_call(
            &timer_, MDNS_QUERY_CHECK_TIMEOUT_NSEC, STATE(mdns_check));
    }
    else if (mdnsClientTrigRefresh_)
    {
        // An mdns_scan() was triggered by the application, start another query
        // immediately.
        return call_immediately(STATE(mdns_query));
    }
    else
    {
        // No more active or pending queries. block mDNS on the STA interface
        // and resume mDNS advertising.
        if (mdnsAdvInhibitStaActive_)
        {
            mdns_disable_sta();
            mdns_adv_inhibit_remove();
            mdnsAdvInhibitStaActive_ = false;
        }
        return sleep_and_call(
            &timer_, MDNS_QUERY_INACTIVE_TIMEOUT_NSEC, STATE(mdns_start));
    }
}

//
// EspIdfWiFi::mdns_adv_inhibit()
//
void EspIdfWiFi::mdns_adv_inhibit()
{
    OSMutexLock locker(this);
    if (!mdnsAdvInhibit_)
    {
        mdnsAdvInhibit_ = true;
        for (auto it = mdnsServices_.begin(); it != mdnsServices_.end(); ++it)
        {
            LOG(VERBOSE, "wifi: mDNS inhibit, service: %s, proto: %s",
                (*it).name_.c_str(), (*it).proto_.c_str());
            ::mdns_service_remove((*it).name_.c_str(), (*it).proto_.c_str());
        }
    }
}

//
// EspIdfWiFi::mdns_adv_inhibit_remove()
//
void EspIdfWiFi::mdns_adv_inhibit_remove()
{
   OSMutexLock locker(this);
    if (mdnsAdvInhibit_)
    {
        mdnsAdvInhibit_ = false;
        for (auto it = mdnsServices_.begin(); it != mdnsServices_.end(); ++it)
        {
            LOG(VERBOSE, "wifi: mDNS inhibit removed, service: %s, proto: %s",
                (*it).name_.c_str(), (*it).proto_.c_str());
            ::mdns_service_add(nullptr, (*it).name_.c_str(),
                (*it).proto_.c_str(), (*it).port_, nullptr, 0);
        }
    }
}

//
// EspIdfWiFi::mdns_disable_sta()
//
void EspIdfWiFi::mdns_disable_sta()
{
#if !defined(CONFIG_MDNS_PREDEF_NETIF_STA)
    OSMutexLock locker(this);
    HASSERT(mdnsStaLockCount_ < 10); // Check for runaway asymmetry.
    if (mdnsStaLockCount_++ == 0)
    {
        ESP_ERROR_CHECK(mdns_unregister_netif(staIface_));
        LOG(VERBOSE, "wifi: STA unregistered on mDNS.");
    }
#endif
}

//
// EspIdfWiFi::try_restore_sta()
//
void EspIdfWiFi::mdns_restore_sta()
{
#if !defined(CONFIG_MDNS_PREDEF_NETIF_STA)
    OSMutexLock locker(this);
    HASSERT(mdnsStaLockCount_ != 0);
    if (--mdnsStaLockCount_ == 0)
    {
        if (staIface_)
        {
            // If the STA interface is not registered by default, register it.
            ESP_ERROR_CHECK(mdns_register_netif(staIface_));
            mdns_event_actions_t action = static_cast<mdns_event_actions_t>(
                MDNS_EVENT_ENABLE_IP4 | MDNS_EVENT_ENABLE_IP6 |
                MDNS_EVENT_ANNOUNCE_IP4 | MDNS_EVENT_ANNOUNCE_IP6);
            ESP_ERROR_CHECK(mdns_netif_action(staIface_, action));
            LOG(VERBOSE, "wifi: STA registered on mDNS.");
        }
    }
#endif
}

//
// EspIdfWiFi::wifi_event_handler()
//
void EspIdfWiFi::wifi_event_handler(esp_event_base_t base, int32_t id, void *data)
{
    if (base != WIFI_EVENT)
    {
        LOG(WARNING, "wifi: wifi_event_handler(), not a WIFI_EVENT.");
        return;
    }

    switch (id)
    {
        default:
            LOG(INFO, "wifi: wifi_event_handler() unknown id: %i", (int)id);
            break;
        case WIFI_EVENT_WIFI_READY:
            started_ = true;
            LOG(INFO, "wifi: WiFi started.");
            break;
        case WIFI_EVENT_STA_START:
        {
            std::string ssid;
            std::string pass;
            uint8_t authmode;
            uint8_t channel;
            last_sta_get(&ssid, &pass, &authmode, &channel);
            if (ssid.empty())
            {
                // No valid "fast" connect parameters.
                esp_wifi_scan_start(&SCAN_CONFIG, false);
                LOG(INFO, "wifi: STA start, scanning...");
            }
            else
            {
                // Try the fast connect parameters first.
                sta_connect(ssid, pass, authmode, channel);
                LOG(INFO, "wifi: STA start, ssid: %s, pass: %s, connecting...",
                    ssid.c_str(), pass.c_str());
            }
            break;
        }
        case WIFI_EVENT_STA_CONNECTED:
        {
            HASSERT(!connected_);
            connected_ = true;
            securityFailure_ = false;
            wifi_event_sta_connected_t *sdata =
                (wifi_event_sta_connected_t*)data;
            LOG(INFO, "wifi: STA connected, channel: %u, authmode: %u",
                sdata->channel, (unsigned)sdata->authmode);
            last_sta_update(std::string((char*)sdata->ssid, sdata->ssid_len),
                staConnectPass_, sdata->authmode, sdata->channel);
            break;
        }
        case WIFI_EVENT_STA_DISCONNECTED:
        {
            HASSERT(connected_);
            connected_ = false;
            wifi_event_sta_disconnected_t *evdata =
                (wifi_event_sta_disconnected_t *)data;
            LOG(INFO, "wifi: STA disconnected ssid=%s reason %d rssi=%d",
                evdata->ssid, evdata->reason, evdata->rssi);

            if (evdata->reason == WIFI_REASON_AUTH_FAIL)
            {
                securityFailure_ = true;
            }

            bool try_fast_reconnect = false;
            {
                // We have to take a mutex because for some reason the
                // IP_EVENT_STA_LOST_IP event does not get delivered when there
                // is an unexpected WiFi disconnect. I guess it is implied?
                OSMutexLock locker(this);
                if (ipAcquiredSta_)
                {
                    ipAcquiredSta_ = false; // Disconnect implies we lost IP.
                    try_fast_reconnect = true; // Try a fast reconnect.
                    mdns_disable_sta();
                }
            }
            if (try_fast_reconnect)
            {
                ip_acquired(IFACE_STA, false);
            }
            if (try_fast_reconnect || fastConnectOnlySta_)
            {
                // First reconnect attempt.
                esp_wifi_connect();
                LOG(INFO, "wifi: STA disconnected, fast reconnect attempt...");
            }
            else
            {
                // Successive reconnect attempts.
                esp_wifi_scan_start(nullptr, false);
                LOG(INFO, "wifi: STA disconnected, scanning...");
            }
            break;
        }
        case WIFI_EVENT_SCAN_DONE:
        {
            /// @todo The newer IDF versions have a new api that can read
            ///       AP records one at a time, which is much more memory
            ///       friendly. Revisit this when updating the IDF version.
            int idx = -1;
            uint16_t number;
            esp_wifi_scan_get_ap_num(&number);
            LOG(INFO, "wifi: Scan done, number of records: %u.", number);
            number = std::min(number, static_cast<uint16_t>(10));

            // This takes a lot of memory. Not the best API design. See note
            // above.
            wifi_ap_record_t *ap_records = new wifi_ap_record_t[10];
            esp_wifi_scan_get_ap_records(&number, ap_records);
            std::string ssid(MAX_SSID_SIZE + 1, '\0');
            std::string pass(MAX_PASS_SIZE + 1, '\0');
            uint8_t sec = WIFI_AUTH_WPA2_PSK;
            uint8_t conn_channel = 0;
            {
                OSMutexLock locker(this);
                scanResults_.clear();
                for (unsigned i = 0; i < number; ++i)
                {
                    // Cache the scan record for the user to read later.
                    scanResults_.emplace_back((char*)ap_records[i].ssid,
                        sec_type_encode(ap_records[i].authmode),
                        ap_records[i].rssi);

                    if (staIface_ && !connected_ && idx < 0)
                    {
                        int index = find_sta_profile(
                            std::string((char*)ap_records[i].ssid));
                        if (index >= 0 &&
                            ap_records[i].authmode >= userCfg_.sta_[index].sec_)
                        {
                            // profile SSID and security mode match.
                            idx = index;
                            ssid = userCfg_.sta_[index].ssid_;
                            pass = userCfg_.sta_[index].pass_;
                            sec = userCfg_.sta_[index].sec_;
                            conn_channel = ap_records[i].primary;
                        }

                    }
                }
            }
            delete ap_records;
            if (staIface_ && !connected_)
            {
                // If in STA mode and not connected, always be trying to make
                // a connection.
                if (idx < 0)
                {
                    // No profile match found, scan again.
                    esp_wifi_scan_start(nullptr, false);
                }
                else
                {
                    // Profile match found, connect.
                    sta_connect(ssid.c_str(), pass.c_str(), sec, conn_channel);
                }
            }
            scan_finished();
            break;
        }
        case WIFI_EVENT_AP_START:
            ipAcquiredAp_ = true;
            ip_acquired(IFACE_AP, true);
            break;
        case WIFI_EVENT_AP_STOP:
            ipAcquiredAp_ = false;
            ip_acquired(IFACE_AP, false);
            break;
        case WIFI_EVENT_AP_STACONNECTED:
        {
            wifi_event_ap_staconnected_t *event =
                (wifi_event_ap_staconnected_t *)data;
            LOG(INFO, "wifi: station " MACSTR " join, AID=%d",
                MAC2STR(event->mac), event->aid);
            ++apClientCount_;
            break;
        }
        case WIFI_EVENT_AP_STADISCONNECTED:
        {
            wifi_event_ap_stadisconnected_t *event =
                (wifi_event_ap_stadisconnected_t *)data;
            LOG(INFO, "wifi: station " MACSTR " leave, AID=%d",
                MAC2STR(event->mac), event->aid);
            if (--apClientCount_ == 0)
            {
                ipLeased_ = false;
            }
            break;
        }
    }
}

//
// EspIdfWiFi::ip_event_handler()
//
void EspIdfWiFi::ip_event_handler(esp_event_base_t base, int32_t id, void *data)
{
    if (base != IP_EVENT)
    {
        LOG(WARNING, "wifi: ip_event_handler(), not a IP_EVENT.");
        return;
    }

    switch (id)
    {
        default:
            LOG(INFO, "wifi: ip_event_handler() unknown id: %i", (int)id);
            break;
        case IP_EVENT_STA_GOT_IP:
        {
            {
                // We have to take a mutex because for some reason the
                // IP_EVENT_STA_LOST_IP event does not get delivered when there
                // is an unexpected WiFi disconnect. I guess it is implied?
                OSMutexLock locker(this);
                HASSERT(ipAcquiredSta_ == false);
                ipAcquiredSta_ = true;
                mdns_restore_sta();
            }
            ip_event_got_ip_t *d = static_cast<ip_event_got_ip_t *>(data);
            char ip_addr[16];
            inet_ntoa_r(d->ip_info.ip.addr, ip_addr, 16);
            LOG(INFO, "wifi: STA got IP: %s", ip_addr);
            ip_acquired(IFACE_STA, true);
            break;
        }
        case IP_EVENT_STA_LOST_IP:
        {
            {
                // We have to take a mutex because for some reason the
                // IP_EVENT_STA_LOST_IP event does not get delivered when there
                // is an unexpected WiFi disconnect. I guess it is implied?
                OSMutexLock locker(this);
                if (ipAcquiredSta_)
                {
                    ipAcquiredSta_ = false;
                    mdns_disable_sta();
                }
            }
            LOG(INFO, "wifi: STA lost IP.");
            ip_acquired(IFACE_STA, false);
            break;
        }
        case IP_EVENT_AP_STAIPASSIGNED:
            ipLeased_ = true;
            break;
    }
}

//
// EspIdfWiFi::init_config()
//
void EspIdfWiFi::init_config()
{
    esp_err_t result = nvs_open(NVS_NAMESPACE_NAME, NVS_READWRITE, &cfg_);
    if (result != ESP_OK)
    {
        LOG(LEVEL_ERROR, "wifi: Error %s opening NVS handle.",
            esp_err_to_name(result));
        return;
    }

    // User configuration.
    size_t len = sizeof(userCfg_);
    result = nvs_get_blob(cfg_, NVS_KEY_USER_NAME, &userCfg_, &len);
    switch(result)
    {
        case ESP_OK:
            if (userCfg_.magic_ == WIFI_CONFIG_INIT_MAGIC &&
                len == sizeof(userCfg_))
            {
                // Already initialized.
                break;
            }
            // Not initialized yet.
            // fall through
        case ESP_ERR_NVS_NOT_FOUND:
            // fall through
        case ESP_ERR_NVS_INVALID_LENGTH:
            // Initialize WiFi user configuration to factory defaults.
            factory_default();
            nvs_set_blob(cfg_, NVS_KEY_USER_NAME, &userCfg_, sizeof(userCfg_));
            nvs_commit(cfg_);
            break;
        default:
            LOG(LEVEL_ERROR, "wifi: Error %s getting userCfg_.",
                esp_err_to_name(result));
            break;
    }

    // Private configuration.
    len = sizeof(privCfg_);
    result = nvs_get_blob(cfg_, NVS_KEY_LAST_NAME, &privCfg_, &len);
    switch (result)
    {
        case ESP_OK:
            if (len == sizeof(privCfg_))
            {
                // Already initialized.
                break;
            }
            // Not initialized yet.
            // fall through
        case ESP_ERR_NVS_NOT_FOUND:
            // fall through
        case ESP_ERR_NVS_INVALID_LENGTH:
            // Reset to factory defaults.
            memset(&privCfg_, 0, sizeof(privCfg_));
            nvs_set_blob(cfg_, NVS_KEY_LAST_NAME, &userCfg_, sizeof(userCfg_));
            nvs_commit(cfg_);
            break;
        default:
            LOG(LEVEL_ERROR, "wifi: Error %s getting privCfg_.",
                esp_err_to_name(result));
            break;
    }
}

//
// EspIdfWiFi::init_wifi()
//
void EspIdfWiFi::init_wifi(WlanRole role)
{
    if (role == WlanRole::DEFAULT_ROLE)
    {
        role = userCfg_.mode_;
    }

    ESP_ERROR_CHECK(esp_netif_init());

    /// @todo Do we need this?
    esp_err_t result = esp_event_loop_create_default();
    if (result != ESP_OK)
    {
        LOG(LEVEL_ERROR, "wifi: esp_event_loop_create_default() failed: %s",
            esp_err_to_name(result));
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, this, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_handler, this, nullptr));

    // Use this to debug wifi connection problems
    // esp_log_level_set("wifi", ESP_LOG_VERBOSE);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));


    // Set and initialize the appropriate role(s).
    switch (role)
    {
        default:
            // Fall through.
        case WlanRole::UNKNOWN:
            // Fall through.
        case WlanRole::AP_STA:
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
            init_softap(userCfg_.ap_.ssid_, userCfg_.ap_.pass_);
            init_sta();
            break;
        case WlanRole::AP:
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
            init_softap(userCfg_.ap_.ssid_, userCfg_.ap_.pass_);
            break;
        case WlanRole::STA:
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
            init_sta();
            break;
    }
    // Start the WiFi.
    ESP_ERROR_CHECK(esp_wifi_start());

    if (apIface_)
    {
        esp_netif_ip_info_t ip_info;
        esp_netif_get_ip_info(
            esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);

        char ip_addr[16];
        inet_ntoa_r(ip_info.ip.addr, ip_addr, 16);
        LOG(INFO, "wifi: Set up softAP with IP: %s", ip_addr);

        LOG(INFO, "wifi: init_softap() finished. SSID:'%s' password:'%s'",
            userCfg_.ap_.ssid_, userCfg_.ap_.pass_);
    }

    // Initialize mDNS.
    ESP_ERROR_CHECK(mdns_init());
    mdns_hostname_set(hostname_);
    mdns_instance_name_set(hostname_);
}

//
// EspIdfWiFi::init_softap()
//
void EspIdfWiFi::init_softap(std::string ssid, std::string pass)
{
    apIface_ = esp_netif_create_default_wifi_ap();
    esp_netif_set_hostname(apIface_, hostname_);

    wifi_config_t conf;
    memset(&conf, 0, sizeof(wifi_config_t));

    conf.ap.max_connection = 4;
    conf.ap.beacon_interval = 100;
    conf.ap.ssid_len = std::min(ssid.size(), (size_t)MAX_SSID_LEN);
    memcpy(conf.ap.ssid, ssid.c_str(), conf.ap.ssid_len);

    size_t pass_len = std::min(pass.size(), (size_t)MAX_PASSPHRASE_LEN);
    if (pass_len == 0)
    {
        conf.ap.authmode = WIFI_AUTH_OPEN;
    }
    else
    {
        memcpy(conf.ap.password, pass.c_str(), pass_len);
        conf.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &conf));
}

//
// EspIdfWiFi::init_sta()
//
void EspIdfWiFi::init_sta()
{
    staIface_ = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(staIface_, hostname_);
    ESP_ERROR_CHECK(esp_netif_dhcpc_start(staIface_));
}

//
// EspIdfWiFi::sta_connect()
//
void EspIdfWiFi::sta_connect(
    std::string ssid, std::string pass, uint8_t authmode, uint8_t channel)
{
    wifi_config_t conf;
    memset(&conf, 0, sizeof(wifi_config_t));

    size_t ssid_len = std::min(ssid.size(), (size_t)MAX_SSID_LEN);
    memcpy(conf.sta.ssid, ssid.c_str(), ssid_len);

    size_t pass_len = std::min(pass.size(), (size_t)MAX_PASSPHRASE_LEN);
    memcpy(conf.sta.password, pass.c_str(), pass_len);
    staConnectPass_ = std::move(pass);

    LOG(INFO, "wifi: STA SSID: %s, PASS: %s", conf.sta.ssid, conf.sta.password);
    if (pass_len)
    {
        conf.sta.threshold.authmode = (wifi_auth_mode_t)authmode;
    }
    else
    {
        conf.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }
    conf.sta.pmf_cfg.required = false;
    conf.sta.scan_method = WIFI_FAST_SCAN;
    conf.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    conf.sta.threshold.rssi = -100;
    conf.sta.channel = channel;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &conf));
    esp_wifi_connect();
}


//
// EspIdfWiFi::last_sta_update()
//
void EspIdfWiFi::last_sta_update(
    std::string ssid, std::string pass, uint8_t authmode, uint8_t channel)
{
    if (ssid != privCfg_.last_.ssid_ || pass != privCfg_.last_.pass_ ||
        authmode != privCfg_.last_.sec_|| channel != privCfg_.channelLast_)
    {
        LOG(INFO, "wifi: Update last STA.");
        strncpy(privCfg_.last_.ssid_, ssid.c_str(), MAX_SSID_SIZE);
        privCfg_.last_.ssid_[MAX_SSID_SIZE] = '\0';
        strncpy(privCfg_.last_.pass_, pass.c_str(), MAX_PASS_SIZE);
        privCfg_.last_.pass_[MAX_PASS_SIZE] = '\0';
        privCfg_.last_.sec_ = authmode;
        privCfg_.channelLast_ = channel;
        nvs_set_blob(cfg_, NVS_KEY_LAST_NAME, &privCfg_, sizeof(privCfg_));
        nvs_commit(cfg_);
    }
}

//
// EspIdfWiFi::last_sta_set()
//
void EspIdfWiFi::last_sta_get(
    std::string *ssid, std::string *pass, uint8_t *authmode, uint8_t *channel)
{
    if (privCfg_.last_.ssid_[0] != '\0' &&
        find_sta_profile(privCfg_.last_.ssid_) >= 0)
    {
        // Last SSID is valid and exists in the profile list.
        *ssid = privCfg_.last_.ssid_;
        *pass = privCfg_.last_.pass_;
        *authmode = privCfg_.last_.sec_;
        *channel = privCfg_.channelLast_;
    }
    else
    {
        ssid->clear();
    }
}

//
// EspIdfWiFi::find_sta_profile()
//
int EspIdfWiFi::find_sta_profile(std::string ssid)
{
    OSMutexLock locker(this);
    for (unsigned i = 0; i < ARRAYSIZE(userCfg_.sta_); ++i)
    {
        if (userCfg_.sta_[i].ssid_ == ssid)
        {
            return i;
        }
    }
    return -1;
}

//
// EspIdfWiFi::sec_type_translate()
//
uint8_t EspIdfWiFi::sec_type_translate(SecurityType sec_type)
{
    switch (sec_type)
    {
        default:
        case SEC_OPEN:
            return WIFI_AUTH_OPEN;
        case SEC_WEP:
            return WIFI_AUTH_WEP;
        case SEC_WPA2:
            return WIFI_AUTH_WPA2_PSK;
    }
}

//
// EspIdfWiFi::sec_type_translate()
//
WiFiInterface::SecurityType EspIdfWiFi::sec_type_encode(uint8_t sec_type)
{
    switch (sec_type)
    {
        default:
        case WIFI_AUTH_OPEN:
            return SEC_OPEN;
        case WIFI_AUTH_WEP:
            return SEC_WEP;
        case WIFI_AUTH_WPA2_PSK:
            return SEC_WPA2;
    }
}