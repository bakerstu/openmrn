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
 * WiFi manager for ESP-IDF platform. Implements WiFiInterface.
 *
 * @author Balazs Racz, extended by Stuart Baker
 * @date 27 Aug 2024, extended starting Jun 17 2025
 */

// Uncomment following line to enable verbose logging.
//#define LOGLEVEL VERBOSE

#include "freertos_drivers/esp_idf/EspIdfWiFi.hxx"

#include <ifaddrs.h>

#include <esp_event.h>
#include <esp_wifi.h>
#include <lwip/inet.h>
#include <lwip/netdb.h>
#include <mdns.h>

/// The name "key" for the AP network interface.
static constexpr char NETIF_KEY_NAME_AP[] = "WIFI_AP_DEF";

/// The name "key" for the AP network interface.
static constexpr char NETIF_KEY_NAME_STA[] = "WIFI_STA_DEF";


/// Scanning parameter configuration.
static const wifi_scan_config_t SCAN_CONFIG = 
{
    .ssid = nullptr,      // any SSID
    .bssid = nullptr,     // any BSSID
    .channel = 0,         // any channel
    .show_hidden = false, // do not include "hidden" SSIDs
    .scan_type = WIFI_SCAN_TYPE_ACTIVE, // active scan
    .scan_time =
    {
        .active =
        {
            .min = 0,   // milliseconds
            .max = 120, // milliseconds
        },
        .passive = 360, // milliseconds
    },
    .home_chan_dwell_time = 30, // milliseconds
    .channel_bitmap =
    {
        .ghz_2_channels = 0,
        .ghz_5_channels = 0,
    },
};

//
// mdns_publish()
//
void mdns_publish(const char *name, const char *service, uint16_t port)
{
    static_cast<EspIdfWiFiBase*>(WiFiInterface::instance())
        ->mdns_service_add(service, port);
}

//
// mdns_unpublish()
//
void mdns_unpublish(const char *name, const char *service)
{
    static_cast<EspIdfWiFiBase*>(WiFiInterface::instance())
        ->mdns_service_remove(service);
}

//
// mdns_scan()
//
void mdns_scan(const char *service)
{
    static_cast<EspIdfWiFiBase*>(WiFiInterface::instance())
        ->mdns_scan(service);
}

//
// mdns_lookup()
//
int mdns_lookup(const char *service, struct addrinfo *hints,
                struct addrinfo **addr)
{
    return static_cast<EspIdfWiFiBase*>(WiFiInterface::instance())
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

        LOG(VERBOSE, "wifi: getifaddrs() ip: %s, netmask: %s",
            ipv4_to_string(ntohl(addr_in[0].sin_addr.s_addr)).c_str(),
            ipv4_to_string(ntohl(addr_in[1].sin_addr.s_addr)).c_str());

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
    HASSERT(ifap);

    esp_netif_t *netif_ap =
        esp_netif_get_handle_from_ifkey(NETIF_KEY_NAME_AP);
    esp_netif_t *netif_sta =
        esp_netif_get_handle_from_ifkey(NETIF_KEY_NAME_STA);
    struct ifaddrs *if_addrs_ap = getifaddrs_helper(netif_ap, "ap");
    struct ifaddrs *if_addrs_sta = getifaddrs_helper(netif_sta, "sta");

    if (if_addrs_ap)
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
// EspIdfWiFiBase::stop()
//
void EspIdfWiFiBase::stop()
{
    initialized_ = false;
    esp_wifi_stop();
}

//
// EspIdfWiFiBase::connect()
//
void EspIdfWiFiBase::connect(
    const char *ssid, const char *pass, SecurityType sec_type)
{
    sta_connect(ssid, pass, sec_type_translate(sec_type));
}

//
// EspIdfWiFiBase::disconnect()
//
void EspIdfWiFiBase::disconnect()
{
    esp_wifi_disconnect();
}

//
// EspIdfWiFiBase::scan()
//
void EspIdfWiFiBase::scan()
{
    esp_wifi_scan_start(&SCAN_CONFIG, false);
}

//
// EspIdfWiFiBase::rssi()
//
int EspIdfWiFiBase::rssi()
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
// EspIdfWiFiBase::factory_reset()
//
void EspIdfWiFiBase::factory_reset()
{
    nvs_handle_t cfg;
    esp_err_t result = nvs_open(NVS_NAMESPACE_NAME, NVS_READWRITE, &cfg);
    if (result != ESP_OK)
    {
        LOG_ERROR("wifi: Error %s opening NVS handle.",
            esp_err_to_name(result));
        return;
    }

    // Clear private configuration.
    memset(&privCfg_, 0, sizeof(privCfg_));
    nvs_erase_key(cfg, NVS_KEY_LAST_NAME);
    nvs_commit(cfg);
    nvs_close(cfg);

    // Default "volatile" values will be put in privCfg_ in init_config_priv().
}

//
// EspIdfWiFiBase::mdns_service_add()
//
void EspIdfWiFiBase::mdns_service_add(const char *service, uint16_t port)
{
    std::string name;
    std::string proto;
    mdns_split(service, &name, &proto);
    LOG(VERBOSE, "wifi: MDNS service add, name: %s, proto: %s, port: %u",
        name.c_str(), proto.c_str(), port);
    OSMutexLock locker(&lock_);
    if (!mdnsAdvInhibit_)
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(::mdns_service_add(
            nullptr, name.c_str(), proto.c_str(), port, nullptr, 0));
    }
    mdnsServices_.emplace_back(std::move(name), std::move(proto), port);
}

//
// EspIdfWiFiBase::mdns_service_remove()
//
void EspIdfWiFiBase::mdns_service_remove(const char *service)
{
    std::string name;
    std::string proto;
    mdns_split(service, &name, &proto);
    LOG(VERBOSE, "wifi: MDNS service remove, service: %s, proto: %s",
        name.c_str(), proto.c_str());
    OSMutexLock locker(&lock_);
    if (!mdnsAdvInhibit_)
    {
        ::mdns_service_remove(name.c_str(), proto.c_str());
    }
    for (auto it = mdnsServices_.begin(); it != mdnsServices_.end(); ++it)
    {
        if (it->name_ == name && it->proto_ == proto)
        {
            mdnsServices_.erase(it);
            return;
        }
    }
}

//
// EspIdfWiFiBase::mdns_lookup()
//
int EspIdfWiFiBase::mdns_lookup(
    const char *service, struct addrinfo *hints, struct addrinfo **addr)
{
    constexpr size_t total_size =
        sizeof(struct addrinfo) + sizeof(struct sockaddr_storage);
    static_assert(
        total_size <= NETDB_ELEM_SIZE, "total_size > NETDB_ELEM_SIZE");

    bool looking = false;
    *addr = nullptr;
    uint32_t now_sec;
    {
        OSMutexLock locker(&lock_);
        now_sec = NSEC_TO_SEC(OSTime::get_monotonic());
        for (auto it = mdnsClientCache_.begin(); it != mdnsClientCache_.end();
            ++it)
        {
            if (it->service_ != service)
            {
                // Not a match.
                continue;
            }

            // Found a potential match. Check for an address list
            looking = true;
            size_t addr_cnt = it->addr_.size();
            if (addr_cnt == 0)
            {
                // No resolved addresses to report.
                continue;
            }

            struct addrinfo *current = *addr;
            struct addrinfo *last = nullptr;

            for (auto lit = it->addr_.begin(); lit != it->addr_.end(); ++lit)
            {
    // The caller is expected to free the struct addrinfo using the method
    // freeaddrinfo. In the lwIP implementation, they use a special buffer pool
    // to free the struct addrinfo to. Therefore, we must also allocate from the
    // same pool.
    #if !defined(LWIP_HDR_MEMP_H)
    #error "lwIP memory pool implementation required and not found."
    #endif
                if ((now_sec - lit->timestamp_) > lit->ttl_)
                {
                    // Address has expired. It will be removed from the list
                    // elsewhere.
                    continue;
                }
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
                LOG(VERBOSE, "wifi: mdns_lookup() address: %s, port: %u",
                    ipv4_to_string(ntohl(ca->addrIn_.sin_addr.s_addr)).c_str(),
                    ntohs(ca->addrIn_.sin_port));
            }
            if (last)
            {
                last->ai_next = nullptr;
            }
            return *addr ? 0 : EAI_MEMORY;
        }
        // Release the mutex.
    }
    if (*addr == nullptr)
    {
        // Not found yet, do a "fast" blocking query.
        std::string name;
        std::string proto;
        mdns_split(service, &name, &proto);

        // Enable mDNS on STA if required.
        bool mdns_adv_inhibited_on_sta = false;
        if (mdnsAdvInhibitSta_)
        {
            mdns_adv_inhibited_on_sta = true;
            mdns_adv_inhibit();
            mdns_restore_sta();
        }

        // Do the blocking query for 2 seconds, only 1 result.
        mdns_result_t *results;
        esp_err_t err =
            mdns_query_ptr(name.c_str(), proto.c_str(), 2000, 1, &results);

        // Disable mDNS on STA if required.
        if (mdns_adv_inhibited_on_sta)
        {
            mdns_disable_sta();
            mdns_adv_inhibit_remove();
        }

        // Check for an error result from the query.
        if (err != ESP_OK)
        {
            LOG_ERROR("wifi: mdns_query_ptr() failed: %s",
                esp_err_to_name(err));
                return EAI_AGAIN;
        }

        if (results)
        {
            // Only return one result.
            struct addrinfo *addr_info =
                (struct addrinfo*)memp_malloc(MEMP_NETDB);
            if (addr_info == nullptr)
            {
                return EAI_MEMORY;
            }
            memset(addr_info, 0, total_size);
                    addr_info->ai_addrlen = sizeof(struct sockaddr_storage);
            addr_info->ai_addr = (struct sockaddr*)((addr_info) + 1);
            struct sockaddr_in *addr_in =
                (struct sockaddr_in*)addr_info->ai_addr;
#if defined(ESP_IDF_WIFI_IPV6)
            struct sockaddr_in6 *addr_in6 =
                (struct sockaddr_in6*)addr_info->ai_addr;
#endif
            mdns_ip_addr_t *current = results->addr;
            while (current)
            {
                if (current->addr.type == ESP_IPADDR_TYPE_V4)
                {
                    addr_info->ai_family = AF_INET;
                    addr_in->sin_family = AF_INET;
                    addr_in->sin_port = htons(results->port);
                    addr_in->sin_len = sizeof(struct sockaddr_in);
                    addr_in->sin_addr.s_addr = current->addr.u_addr.ip4.addr;
                    *addr = addr_info;
                    break;
                }
#if defined(ESP_IDF_WIFI_IPV6)
                else if (current->addr.type == ESP_IPADDR_TYPE_V6)
                {
                    addr_info->ai_family = AF_INET6;
                    addr_in6->sin6_family = AF_INET6;
                    addr_in6->sin6_port = htons(results->port);
                    addr_in6->sin6_len = sizeof(struct sockaddr_in6);
                    memcpy(addr_in6->sin6_addr.un.u32_addr,
                        current->addr.u_addr.ip6.addr,
                        sizeof(addr_in6->sin6_addr.un.u32_addr));
                    *addr = addr_info;
                    break;
                }
#else
#endif
                current = current->next;
            }
            if (*addr == nullptr)
            {
                // No result actually found.
                freeaddrinfo(addr_info);
            }
            else
            {
                // Result found, add it to the cache.
                OSMutexLock locker(&lock_);
                mdnsClientCache_.emplace_back(service);
                MDNSCacheAddr ca;
                ca.timestamp_ = now_sec;
                ca.ttl_ = results->ttl ? results->ttl : 1;
                ca.family_ = (*addr)->ai_family;
                ca.port_ = results->port;
                if ((*addr)->ai_family == AF_INET)
                {
                    memcpy(&ca.addrIn_, addr_in, sizeof(ca.addrIn_));
                }
#if defined(ESP_IDF_WIFI_IPV6)
                else if ((*addr)->ai_family == AF_INET6)
                {
                    memcpy(&ca.addrIn6_, addr_in6, sizeof(ca.addrIn6_));
                }
#endif
                auto it = mdnsClientCache_.end() - 1;
                it->addr_.emplace_front(ca);
            }
        } // if (results)
        mdns_query_results_free(results);
    }
    if (*addr == nullptr && !looking)
    {
        // Start background scanning for the service.
        mdns_scan(service);
    }
    return EAI_AGAIN;
}

//
// EspIdfWiFiBase::mdns_scan()
//
void EspIdfWiFiBase::mdns_scan(const char *service)
{
    bool match = false;
    OSMutexLock locker(&lock_);
    for (auto it = mdnsClientCache_.begin(); it != mdnsClientCache_.end(); ++it)
    {
        if (it->service_ == service)
        {
            match = true;
        }
    }
    if (!match)
    {
        // Start looking for a new service.
        mdnsClientCache_.emplace_back(service);
    }
    mdns_scanning_start_or_trigger_refresh();
}

//
// EspIdfWiFiBase::MDNSCacheItem::reset()
//
void EspIdfWiFiBase::MDNSCacheItem::reset(void *handle)
{
    if (searchHandle_ && handle != searchHandle_)
    {
        mdns_query_async_delete((mdns_search_once_t*)searchHandle_);
    }
    searchHandle_ = handle;
}

//
// EspIdfWiFiBase::mdns_start()
//
StateFlowBase::Action EspIdfWiFiBase::mdns_start()
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
// EspIdfWiFiBase::mdns_query()
//
StateFlowBase::Action EspIdfWiFiBase::mdns_query()
{
    OSMutexLock locker(&lock_);
    mdnsClientTrigRefresh_ = false;
    for (auto it = mdnsClientCache_.begin(); it != mdnsClientCache_.end(); ++it)
    {
        std::string name;
        std::string proto;
        mdns_split(it->service_.c_str(), &name, &proto);
        it->reset((void*)mdns_query_async_new(
            nullptr, name.c_str(), proto.c_str(), MDNS_TYPE_PTR,
            MDNS_QUERY_ASYNC_NEW_TIMEOUT_MSEC, 3, nullptr));
        LOG(VERBOSE, "wifi: mDNS new search query: %s.%s",
            name.c_str(), proto.c_str());
    }

    return sleep_and_call(&timer_,
        MDNS_QUERY_CHECK_TIMEOUT_NSEC, STATE(mdns_check));
}

//
// EspIdfWiFiBase::mdns_check()
//
StateFlowBase::Action EspIdfWiFiBase::mdns_check()
{
    bool search_still_active = false;
    OSMutexLock locker(&lock_);
    uint32_t now_sec = NSEC_TO_SEC(OSTime::get_monotonic());
    // Loop through all of the service clients.
    for (auto it = mdnsClientCache_.begin(); it != mdnsClientCache_.end(); ++it)
    {
        if (it->get() == nullptr)
        {
            // No active search for this cache entry.
            LOG(VERBOSE, "wifi: mDNS no active search for this cache entry.");
            continue;
        }
        mdns_result_t *mdns_result = nullptr;
        bool result = mdns_query_async_get_results(
            (mdns_search_once_t*)it->get(), 0, &mdns_result, nullptr);
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
            it->reset();
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
                for (auto ait = it->addr_.begin();
                    !duplicate && ait != it->addr_.end(); ++ait)
                {
                    duplicate = ait->family_ == cache_addr.family_;
                    duplicate &= ait->port_ == cache_addr.port_;
                    if (cache_addr.family_ == AF_INET)
                    {
                        duplicate &= ait->addrcmp(addr->addr.u_addr.ip4.addr);
                    }
#if defined(ESP_IDF_WIFI_IPV6)
                    else if (cache_addr.family_ == AF_INET6)
                    {
                        duplicate &= ait->addr6cmp(addr->addr.u_addr.ip6.addr);
                    }
#endif
                    if (duplicate)
                    {
                        // Refresh the timestamp and ttl.
                        ait->timestamp_ = now_sec;
                        ait->ttl_ = mdns_result->ttl ? mdns_result->ttl : 1;
                    }
                }
                if (!duplicate)
                {
                    if (cache_addr.family_ == AF_INET)
                    {
                        cache_addr.set_addr(addr->addr.u_addr.ip4.addr);
                        cache_addr.addrIn_.sin_port = cache_addr.port_;
                        LOG(VERBOSE, "wifi: mDNS service discovered: %s\n"
                            "     %s %s:%u", cur_result->hostname,
                            it->service_.c_str(),
                            ipv4_to_string(
                                ntohl(cache_addr.get_addr())).c_str(),
                            ntohs(cache_addr.port_));
                    }
#if defined(ESP_IDF_WIFI_IPV6)
                    else if (cache_addr.family_ == AF_INET6)
                    {
                        cache_addr.set_addr6(addr->addr.u_addr.ip6.addr);
                        cache_addr.addrIn6_.sin6_port = cache_addr.port_;
                        char ip6_str[INET6_ADDRSTRLEN];
                        LOG(VERBOSE, "wifi: mDNS service discovered: %s\n"
                            "     %s %s:%u", cur_result->hostname,
                            it->service_.c_str(),
                            inet_ntop(AF_INET6, cache_addr.get_addr6(), ip6_str,
                                INET6_ADDRSTRLEN),
                            ntohs(cache_addr.port_));
                    }
#endif
                    if (it->addr_.size() >= MDNS_RESULT_COUNT_MAX)
                    {
                        LOG(VERBOSE,
                            "wifi: mDNS service evict old cache entry");
                        it->addr_.pop_back();
                    }
                    cache_addr.timestamp_ = now_sec;
                    cache_addr.ttl_ = mdns_result->ttl ? mdns_result->ttl : 1;
                    it->addr_.emplace_front(cache_addr);
                }
                else
                {
                    LOG(VERBOSE, "wifi: mDNS service discovered duplicate.");
                }
            }
        }
        // Prune any expired entries.
        for (auto ait = it->addr_.begin(); ait != it->addr_.end(); )
        {
            if ((now_sec - ait->timestamp_) > ait->ttl_)
            {
                ait = it->addr_.erase(ait);
            }
            else
            {
                ++ait;
            }
        }
        mdns_query_results_free(mdns_result);
        it->reset();
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
        // No more active or pending queries. Block mDNS on the STA interface
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
// EspIdfWiFiBase::mdns_adv_inhibit()
//
void EspIdfWiFiBase::mdns_adv_inhibit()
{
    OSMutexLock locker(&lock_);
    if (!mdnsAdvInhibit_)
    {
        mdnsAdvInhibit_ = true;
        for (auto it = mdnsServices_.begin(); it != mdnsServices_.end(); ++it)
        {
            LOG(VERBOSE, "wifi: mDNS inhibit, service: %s, proto: %s",
                it->name_.c_str(), it->proto_.c_str());
            ::mdns_service_remove(it->name_.c_str(), it->proto_.c_str());
        }
    }
}

//
// EspIdfWiFiBase::mdns_adv_inhibit_remove()
//
void EspIdfWiFiBase::mdns_adv_inhibit_remove()
{
   OSMutexLock locker(&lock_);
    if (mdnsAdvInhibit_)
    {
        mdnsAdvInhibit_ = false;
        for (auto it = mdnsServices_.begin(); it != mdnsServices_.end(); ++it)
        {
            LOG(VERBOSE, "wifi: mDNS inhibit removed, service: %s, proto: %s",
                it->name_.c_str(), it->proto_.c_str());
            ::mdns_service_add(nullptr, it->name_.c_str(),
                it->proto_.c_str(), it->port_, nullptr, 0);
        }
    }
}

//
// EspIdfWiFiBase::mdns_disable_sta()
//
void EspIdfWiFiBase::mdns_disable_sta()
{
#if !defined(CONFIG_MDNS_PREDEF_NETIF_STA)
    OSMutexLock locker(&lock_);
    HASSERT(mdnsStaLockCount_ < 10); // Check for runaway asymmetry.
    if (mdnsStaLockCount_++ == 0)
    {
        ESP_ERROR_CHECK(mdns_unregister_netif(staIface_));
        LOG(VERBOSE, "wifi: STA unregistered on mDNS.");
    }
#endif
    LOG(VERBOSE, "wifi: mdns_disable_sta(): %u", mdnsStaLockCount_);
}

//
// EspIdfWiFiBase::try_restore_sta()
//
void EspIdfWiFiBase::mdns_restore_sta()
{
#if !defined(CONFIG_MDNS_PREDEF_NETIF_STA)
    OSMutexLock locker(&lock_);
    HASSERT(mdnsStaLockCount_ > -10); // Check for runaway asymmetry.
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
    LOG(VERBOSE, "wifi: mdns_retore_sta(): %u", mdnsStaLockCount_);
}

//
// EspIdfWiFiBase::mdns_scanning_start_or_trigger_refresh()
//
void EspIdfWiFiBase::mdns_scanning_start_or_trigger_refresh()
{
    if (!mdnsClientStarted_)
    {
        // Start the client if not already started.
        mdnsClientStarted_ = true;
        notify();
        LOG(VERBOSE, "wifi: mdns_scan() start.");
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
        LOG(VERBOSE, "wifi: mdns_scan() trigger.");
    }
}

//
// EspIdfWiFiBase::wifi_event_handler()
//
void EspIdfWiFiBase::wifi_event_handler(
    esp_event_base_t base, int32_t id, void *data)
{
    HASSERT(base == WIFI_EVENT);

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
            LOG(INFO, "wifi: STA started.");
            if (!sta_connect_fast() && !fastConnectOnlySta_)
            {
                // No valid "fast" connect parameters.
                esp_wifi_scan_start(&SCAN_CONFIG, false);
                LOG(VERBOSE, "wifi: STA start, scanning...");
            }
            break;
        }
        case WIFI_EVENT_HOME_CHANNEL_CHANGE:
        {
            wifi_event_home_channel_change_t *cdata =
                (wifi_event_home_channel_change_t*)data;
            if (connected_)
            {
                last_sta_update_channel(cdata->new_chan);
            }
            LOG(INFO, "wifi: Home channel change, channel: %u",
                cdata->new_chan);
            break;
        }
        case WIFI_EVENT_STA_CONNECTED:
        {
            HASSERT(!connected_);
            connected_ = true;
            wifi_event_sta_connected_t *sdata =
                (wifi_event_sta_connected_t*)data;
            LOG(INFO, "wifi: STA connected, channel: %u, authmode: %u",
                sdata->channel, (unsigned)sdata->authmode);
            // There is a very small chance for a race condition whereby another
            // call "sta_connect()" call has been made with a different password
            // after this connection has been established. The chances are very
            // small and the harm is very limited to the next fast connect
            // attempt failing.
            std::string ssid((char*)sdata->ssid, sdata->ssid_len);
            last_sta_update(
                ssid, staConnectPass_, sdata->authmode, sdata->channel);

            // Register a callback to run on the passed in service executor.
            CallbackExecutable *e = new CallbackExecutable(std::bind(
                &EspIdfWiFiBase::wlan_connected, this, true, CONNECT_OK,
                std::move(ssid)));
            service()->executor()->add(e);
            break;
        }
        case WIFI_EVENT_STA_DISCONNECTED:
        {
            // Note: It is possible to get here when we are already
            //       "disconnected". Therefore, we should not assert on
            //       connected_ == true. Every failed connection "attempt"
            //       lands us here.
            connected_ = false;
            wifi_event_sta_disconnected_t *evdata =
                (wifi_event_sta_disconnected_t *)data;
            LOG(INFO, "wifi: STA disconnected ssid=%s reason %d rssi=%d",
                evdata->ssid, evdata->reason, evdata->rssi);

            bool try_fast_reconnect = false;
            // For some reason the IP_EVENT_STA_LOST_IP event does not get
            // delivered when there is an unexpected WiFi disconnect. I guess
            // it is implied?
            if (ipAcquiredSta_)
            {
                ipAcquiredSta_ = false; // Disconnect implies we lost IP.
                try_fast_reconnect = true; // Try a fast reconnect.
                mdns_disable_sta();
                // Register a callback to run on the passed in service executor.
                CallbackExecutable *e = new CallbackExecutable(std::bind(
                    &EspIdfWiFiBase::ip_acquired, this, IFACE_STA, false));
                service()->executor()->add(e);
            }
            std::string ssid((char*)evdata->ssid, evdata->ssid_len);

            // Register a callback to run on the passed in service executor.
            ConnectionResult reason = connection_result_encode(evdata->reason);
            CallbackExecutable *e = new CallbackExecutable(std::bind(
                &EspIdfWiFiBase::wlan_connected, this, false, reason,
                std::move(ssid)));
            service()->executor()->add(e);

            if (!initialized_)
            {
                // This is likely a "stop" condition. Do not try to reconnect
                // or scan for APs. Otherwise, there will be a crash.
                break;
            }
            if (try_fast_reconnect || fastConnectOnlySta_)
            {
                // First reconnect attempt (use last channel), or forced "fast"
                // connect (use any channel if not first attempt).
                sta_connect_fast(try_fast_reconnect);
                LOG(VERBOSE, "wifi: STA disconnected, fast reconnect attempt...");
            }
            else
            {
                // Successive reconnect attempts.
                esp_wifi_scan_start(nullptr, false);
                LOG(VERBOSE, "wifi: STA disconnected, scanning...");
            }
            break;
        }
        case WIFI_EVENT_SCAN_DONE:
        {
            collect_scan_results();

            if (staIface_ && !connected_)
            {
                // Not currently connected, try to find a match to connect.
                int match_index = -1;
                std::string ssid;
                std::string pass;
                uint8_t sec;
                uint8_t conn_channel;

                // Look for a result that matches one of our profiles.
                for (auto it = scanResults_.begin();
                    it != scanResults_.end(); ++it)
                {
                    int idx = find_sta_profile(it->ssid, &pass, &sec);
                    if (idx >= 0 && sec_type_translate(it->secType) >= sec)
                    {
                        // Found a profile match, capture the credentials to
                        // connect below.
                        match_index = idx;
                        ssid = it->ssid;
                        sec = sec_type_translate(it->secType);
                        conn_channel = it->channel;
                        LOG(INFO, "wifi: Found match, ssid: %s, sec: %u",
                            ssid.c_str(), sec);
                        break;
                    }
                }

                // If in STA mode and not connected, always be trying to make
                // a connection.
                if (match_index < 0)
                {
                    /// @todo If we get into a situation where we are
                    ///       continuously scanning forever, does this effect
                    ///       the quality of the AP or BLE performance, since
                    ///       there is only one radio? Not sure. Need to do
                    ///       some practical testing. Perhaps there should be
                    ///       some delay between scans.
                    // No profile match found, scan again.
                    esp_wifi_scan_start(nullptr, false);
                }
                else
                {
                    // Profile match found, connect.
                    sta_connect(ssid.c_str(), pass.c_str(), sec, conn_channel);
                }
            }
            // Register a callback to run on the passed in service executor.
            CallbackExecutable *e = new CallbackExecutable(std::bind(
                &EspIdfWiFiBase::scan_finished, this));
            service()->executor()->add(e);
            break;
        }
        case WIFI_EVENT_AP_START:
        {
            ipAcquiredAp_ = true;
            // Register a callback to run on the passed in service executor.
            CallbackExecutable *e = new CallbackExecutable(std::bind(
                &EspIdfWiFiBase::ip_acquired, this, IFACE_AP, true));
            service()->executor()->add(e);

            esp_netif_ip_info_t ip_info;
            esp_netif_get_ip_info(
                esp_netif_get_handle_from_ifkey(NETIF_KEY_NAME_AP), &ip_info);

            char ip_addr[16];
            inet_ntoa_r(ip_info.ip.addr, ip_addr, 16);
            LOG(INFO, "wifi: Set up softAP with IP: %s", ip_addr);
            break;
        }
        case WIFI_EVENT_AP_STOP:
        {
            ipAcquiredAp_ = false;
            // Register a callback to run on the passed in service executor.
            CallbackExecutable *e = new CallbackExecutable(std::bind(
                &EspIdfWiFiBase::ip_acquired, this, IFACE_AP, false));
            service()->executor()->add(e);
            break;
        }
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
// EspIdfWiFiBase::ip_event_handler()
//
void EspIdfWiFiBase::ip_event_handler(
    esp_event_base_t base, int32_t id, void *data)
{
    HASSERT(base == IP_EVENT);

    switch (id)
    {
        default:
            LOG(INFO, "wifi: ip_event_handler() unknown id: %i", (int)id);
            break;
        case IP_EVENT_STA_GOT_IP:
        {
            ip_event_got_ip_t *d = static_cast<ip_event_got_ip_t *>(data);
            HASSERT(ipAcquiredSta_ == false);
            ipAcquiredSta_ = true;
            mdns_restore_sta();
            {
                OSMutexLock locker(&lock_);
                if (mdnsClientCache_.size())
                {
                    // Trigger an immediate scan in order to refresh the cache.
                    mdns_scanning_start_or_trigger_refresh();
                }
            }
            char ip_addr[16];
            inet_ntoa_r(d->ip_info.ip.addr, ip_addr, 16);
            LOG(INFO, "wifi: STA got IP: %s", ip_addr);
            // Register a callback to run on the passed in service executor.
            CallbackExecutable *e = new CallbackExecutable(std::bind(
                &EspIdfWiFiBase::ip_acquired, this, IFACE_STA, true));
            service()->executor()->add(e);
            break;
        }
        case IP_EVENT_STA_LOST_IP:
        {
            if (ipAcquiredSta_)
            {
                ipAcquiredSta_ = false;
                mdns_disable_sta();
                LOG(INFO, "wifi: STA lost IP.");
                // Register a callback to run on the passed in service executor.
                CallbackExecutable *e = new CallbackExecutable(std::bind(
                    &EspIdfWiFiBase::ip_acquired, this, IFACE_STA, false));
                service()->executor()->add(e);
            }
            break;
        }
        case IP_EVENT_AP_STAIPASSIGNED:
            LOG(INFO, "wifi: Soft-AP assigned out IP address.");
            ipLeased_ = true;
            break;
    }
}

//
// EspIdfWiFiBase::collect_scan_results()
//
void EspIdfWiFiBase::collect_scan_results()
{
    uint16_t number;
    esp_wifi_scan_get_ap_num(&number);
    LOG(INFO, "wifi: Scan done, number of records: %u.", number);

    OSMutexLock locker(&lock_);
    scanResults_.clear();
    scanResultsEntryIndex_ = INT_MAX;
    scanResultsEntry_ = scanResults_.end();
    do
    {
        wifi_ap_record_t ap_record;
        if (esp_wifi_scan_get_ap_record(&ap_record) != ESP_OK)
        {
            // No more results, stop processing.
            LOG(VERBOSE, "wifi: No more scan results.");
            break;
        }
        if (ap_record.ssid[0] == '\0')
        {
            // "Hidden SSID, ignore."
            continue;
        }
        if (prune_duplicate_ap_scan_results_by_rssi())
        {
            // Pruning of duplicates is enabled. Look for a
            // duplicate.
            bool duplicate = false;
            for (auto it = scanResults_.begin(); it != scanResults_.end(); ++it)
            {
                if (it->ssid == (char*)ap_record.ssid)
                {
                    // Duplicate. Since results are presented in
                    // highest SSID first, we can ignore this
                    // result.
                    duplicate = true;
                    LOG(VERBOSE, "wifi: Duplicate scan result, ssid: %s",
                        it->ssid.c_str());
                    break;
                }
            }
            if (duplicate)
            {
                // Duplicate, move on to the next record.
                continue;
            }
        }
        // Archive the result for the user.
        scanResults_.emplace_back((char*)ap_record.ssid,
            ap_record.rssi, sec_type_encode(ap_record.authmode),
            ap_record.primary);
        LOG(VERBOSE, "wifi: Added scan result, ssid: %s, sec: %u, chan: %u",
            scanResults_.back().ssid.c_str(), scanResults_.back().secType,
            scanResults_.back().channel);
    } while (scanResults_.size() < max_ap_scan_results());

    // Free any remaining scan results.
    esp_wifi_clear_ap_list();
}

//
// EspIdfWiFiBase::init_config_priv()
//
void EspIdfWiFiBase::init_config_priv()
{
    nvs_handle_t cfg;
    esp_err_t result = nvs_open(NVS_NAMESPACE_NAME, NVS_READWRITE, &cfg);
    if (result != ESP_OK)
    {
        LOG_ERROR("wifi: Error %s opening NVS handle.",
            esp_err_to_name(result));
        return;
    }

    // Private configuration.
    memset(&privCfg_, 0, sizeof(privCfg_));
    size_t len = sizeof(privCfg_);
    result = nvs_get_blob(cfg, NVS_KEY_LAST_NAME, &privCfg_, &len);
    switch (result)
    {
        case ESP_OK:
            if (!config_priv_reset_allowed())
            {
                // Reset of the private configuration is not allowed. No reason
                // to check if it was read out in tact.
                break;
            }
            if (privCfg_.magic_ == PRIV_CONFIG_INIT_MAGIC &&
                len == sizeof(privCfg_))
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
            LOG(INFO, "wifi: Reset private config.");
            memset(&privCfg_, 0, sizeof(privCfg_));
            privCfg_.magic_ = PRIV_CONFIG_INIT_MAGIC;
            nvs_set_blob(cfg, NVS_KEY_LAST_NAME, &privCfg_, sizeof(privCfg_));
            nvs_commit(cfg);
            break;
        default:
            LOG_ERROR("wifi: Error %s getting privCfg_.",
                esp_err_to_name(result));
            break;
    }
    nvs_close(cfg);

    if (privCfg_.last_.ssid_[0] == '\0')
    {
        LOG(VERBOSE, "wifi: No fast connect credentials.");
        // There are no "last" STA credentials to use for fast connect. Set the
        // default STA as the last connected STA, but do not commit it to
        // non-volatile storage. It will be commited to non-volatile storage
        // only once a connection is actually successful.
        str_populate<MAX_SSID_SIZE>(privCfg_.last_.ssid_, default_sta_ssid());
        str_populate<MAX_PASS_SIZE>(
            privCfg_.last_.pass_, default_sta_password());
        privCfg_.last_.sec_ = privCfg_.last_.pass_[0] == '\0' ?
            sec_type_translate(SEC_OPEN) : sec_type_translate(SEC_WPA2);
        privCfg_.channelLast_ = 0; // Any channel.
    }
}

//
// EspIdfWiFiBase::init_wifi()
//
void EspIdfWiFiBase::init_wifi(WlanRole role)
{
    if (role == WlanRole::DEFAULT_ROLE)
    {
        role = get_role();
    }

    ESP_ERROR_CHECK(esp_netif_init());

    /// @todo Do we need this?
    esp_err_t result = esp_event_loop_create_default();
    if (result != ESP_OK)
    {
        LOG_ERROR("wifi: esp_event_loop_create_default() failed: %s",
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

    std::string ap_ssid;
    std::string ap_pass;

    // Set and initialize the appropriate role(s).
    switch (role)
    {
        default:
            // Fall through.
        case WlanRole::UNKNOWN:
            // Fall through.
        case WlanRole::AP_STA:
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
            get_ap_config_password(&ap_ssid, &ap_pass);
            init_softap(std::move(ap_ssid), std::move(ap_pass));
            init_sta();
            break;
        case WlanRole::AP:
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
            get_ap_config_password(&ap_ssid, &ap_pass);
            init_softap(std::move(ap_ssid), std::move(ap_pass));
            break;
        case WlanRole::STA:
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
            init_sta();
            break;
    }
    // Start the WiFi.
    ESP_ERROR_CHECK(esp_wifi_start());

    // Initialize mDNS.
    ESP_ERROR_CHECK(mdns_init());
    mdns_hostname_set(hostname_.c_str());
    mdns_instance_name_set(hostname_.c_str());
    initialized_ = true;
}

//
// EspIdfWiFiBase::init_softap()
//
void EspIdfWiFiBase::init_softap(std::string ssid, std::string pass)
{
    LOG(VERBOSE, "wifi: init_softap(), ssid: %s", ssid.c_str());
    apIface_ = esp_netif_create_default_wifi_ap();
    esp_netif_set_hostname(apIface_, hostname_.c_str());

    wifi_config_t conf;
    memset(&conf, 0, sizeof(wifi_config_t));

    conf.ap.max_connection = max_ap_client_connections();
    // Beacon interval is in units of milliseconds. 100ms beacon interval is
    // pretty universally standard, and it would be extremely unusual to use
    // anything else.
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
// EspIdfWiFiBase::init_sta()
//
void EspIdfWiFiBase::init_sta()
{
    staIface_ = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(staIface_, hostname_.c_str());
    ESP_ERROR_CHECK(esp_netif_dhcpc_start(staIface_));
}

//
// EspIdfWiFiBase::sta_connect()
//
void EspIdfWiFiBase::sta_connect(
    std::string ssid, std::string pass, uint8_t authmode, uint8_t channel)
{
    wifi_config_t conf;
    memset(&conf, 0, sizeof(wifi_config_t));

    size_t ssid_len = std::min(ssid.size(), (size_t)MAX_SSID_LEN);
    memcpy(conf.sta.ssid, ssid.c_str(), ssid_len);

    size_t pass_len = std::min(pass.size(), (size_t)MAX_PASSPHRASE_LEN);
    memcpy(conf.sta.password, pass.c_str(), pass_len);

    LOG(VERBOSE, "wifi: STA SSID: %s", conf.sta.ssid);

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
    conf.sta.threshold.rssi = STA_CONNECT_RSSI_THRESHOLD;
    conf.sta.channel = channel;

    {
        OSMutexLock locker(&lock_);
        staConnectPass_ = std::move(pass);
    }
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &conf));
    esp_wifi_connect();
}

//
// EspIdfWiFiBase::sta_connect_fast()
//
bool EspIdfWiFiBase::sta_connect_fast(bool last_channel)
{
    if (privCfg_.last_.ssid_[0] != '\0')
    {
        uint8_t channel = last_channel ? privCfg_.channelLast_ : 0;
        sta_connect(privCfg_.last_.ssid_, privCfg_.last_.pass_,
            privCfg_.last_.sec_, channel);
        LOG(INFO,
            "wifi: STA fast connect, ssid: %s, channel %u, connecting...",
            privCfg_.last_.ssid_, channel);
        return true;
    }
    return false;
}

//
// EspIdfWiFiBase::last_sta_update()
//
void EspIdfWiFiBase::last_sta_update(
    std::string ssid, std::string pass, uint8_t authmode, uint8_t channel)
{
    if (ssid != privCfg_.last_.ssid_ || pass != privCfg_.last_.pass_ ||
        authmode != privCfg_.last_.sec_|| channel != privCfg_.channelLast_)
    {
        nvs_handle_t cfg;
        esp_err_t result = nvs_open(NVS_NAMESPACE_NAME, NVS_READWRITE, &cfg);
        if (result == ESP_OK)
        {
            LOG(VERBOSE, "wifi: Update last STA.");
            str_populate<MAX_SSID_SIZE>(privCfg_.last_.ssid_, ssid.c_str());
            str_populate<MAX_PASS_SIZE>(privCfg_.last_.pass_, pass.c_str());
            privCfg_.last_.sec_ = authmode;
            privCfg_.channelLast_ = channel;

            nvs_set_blob(cfg, NVS_KEY_LAST_NAME, &privCfg_, sizeof(privCfg_));
            nvs_commit(cfg);
            nvs_close(cfg);
        }
    }
}

//
// EspIdfWiFiBase::last_sta_update_channel()
//
void EspIdfWiFiBase::last_sta_update_channel(uint8_t channel)
{
    if (channel != privCfg_.channelLast_)
    {
        nvs_handle_t cfg;
        esp_err_t result = nvs_open(NVS_NAMESPACE_NAME, NVS_READWRITE, &cfg);
        if (result == ESP_OK)
        {
            LOG(VERBOSE, "wifi: Update last STA channel only.");
            privCfg_.channelLast_ = channel;

            nvs_set_blob(cfg, NVS_KEY_LAST_NAME, &privCfg_, sizeof(privCfg_));
            nvs_commit(cfg);
            nvs_close(cfg);
        }
    }
}

//
// EspIdfWiFiBase::sec_type_translate()
//
uint8_t EspIdfWiFiBase::sec_type_translate(SecurityType sec_type)
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
        case SEC_WPA3:
            return WIFI_AUTH_WPA3_PSK;
    }
}

//
// EspIdfWiFiBase::sec_type_encode()
//
WiFiInterface::SecurityType EspIdfWiFiBase::sec_type_encode(uint8_t sec_type)
{
    switch (sec_type)
    {
        default:
        case WIFI_AUTH_OPEN:
            return SEC_OPEN;
        case WIFI_AUTH_WEP:
            return SEC_WEP;
        case WIFI_AUTH_WPA2_PSK:
            // fall through
        case WIFI_AUTH_WPA_WPA2_PSK:
            return SEC_WPA2;
        case WIFI_AUTH_WPA3_PSK:
            // fall through
        case WIFI_AUTH_WPA2_WPA3_PSK:
            return SEC_WPA3;
    }
}

//
// EspIdfWiFiBase::connection_result_encode()
//
WiFiInterface::ConnectionResult EspIdfWiFiBase::connection_result_encode(
    uint8_t reason)
{
    switch (reason)
    {
        default:
            return WiFiInterface::CONNECT_UNKNOWN;
        case 0:
            return WiFiInterface::CONNECT_OK;
        case WIFI_REASON_AUTH_EXPIRE:
            // fall through
        case WIFI_REASON_AUTH_LEAVE:
            // fall through
        case WIFI_REASON_AUTH_FAIL:
            // fall through
        case WIFI_REASON_NOT_AUTHED:
            // fall through
        case WIFI_REASON_802_1X_AUTH_FAILED:
            // fallthrough
        case WIFI_REASON_ASSOC_NOT_AUTHED:
            return WiFiInterface::AUTHENTICATION_FAILED;
        case WIFI_REASON_ASSOC_EXPIRE:
            // fall through
        case WIFI_REASON_ASSOC_TOOMANY:
            // fall through
        case WIFI_REASON_ASSOC_LEAVE:
            // fall through
        case WIFI_REASON_ASSOC_FAIL:
            // fall through
        case WIFI_REASON_ASSOC_COMEBACK_TIME_TOO_LONG:
            return WiFiInterface::ASSOCIATION_FAILED;
    }
}
