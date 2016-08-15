/** \copyright
 * Copyright (c) 2016, Stuart W Baker
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
 * \file CC32xxWiFi.hxx
 * This file instantiates and initializes the CC32xx Wi-Fi.
 *
 * @author Stuart W. Baker
 * @date 18 March 2016
 */

#ifndef _FREERTOS_DRIVERS_NET_C32XX_CC32XXWIFI_HXX_
#define _FREERTOS_DRIVERS_NET_C32XX_CC32XXWIFI_HXX_

#include <functional>
#include <vector>

#include "os/OS.hxx"

class CC32xxSocket;

/** Provides the startup and mantainance methods for configuring and using the
 * CC32xx Wi-Fi stack.  This is designed to be a singleton.  It should only
 * be instantiated once.
 */
class CC32xxWiFi
{
public:
    /** Security types.
     */
    enum SecurityType
    {
        SEC_OPEN, /**< open (no security) */
        SEC_WEP,  /**< WEP security mode */
        SEC_WPA2, /**< WPA2 security mode */
    };

    /** metadata for a WLAN netowrk entry.
     */
    struct WlanNetworkEntry
    {
        char ssid[33]; /**< SSID of AP */
        SecurityType sec_type; /**< security type of the AP */
        int rssi; /**< receive signal strength indicator of the AP */
    };

    /** Constructor.
     */
    CC32xxWiFi();

    /** Destructor.
     */
    ~CC32xxWiFi()
    {
    }

    /** Startup the Wi-Fi.
     */
    void start();

    /** Stops the Wi-Fi in preparation for a reboot. TODO: does this need to be
     * called from a critical section?
     */
    void stop();

    /** Connect to access point.
     * @param ssid access point ssid
     * @param security_key access point security key
     * @param security_type access point security type, options:\n
     *                          - SL_SEC_TYPE_OPEN
     *                          - SL_SEC_TYPE_WEP
     *                          - SL_SEC_TYPE_WPA_WPA2
     *                          - SL_SEC_TYPE_WPA_ENT
     *                          - SL_SEC_TYPE_WPS_PBC
     *                          - SL_SEC_TYPE_WPS_PIN 
     */
    void wlan_connect(const char *ssid, const char* security_key,
                      uint8_t security_type);


    /** @return true if the wlan interface is ready to establish outgoing
     * connections. */
    bool wlan_ready()
    {
        return connected && ipAquired;
    }

    /** Updates the blinker based on connection state. Noop if wlan_ready()
     * returns true.*/
    void connecting_update_blinker();

    /** Get the singleton instance pointer.
     * @return singleton instance pointer
     */
    static CC32xxWiFi *instance()
    {
        HASSERT(instance_);
        return instance_;
    }

    /** Add a saved WLAN profile.
     * @param ssid WLAN SSID of the profile to save
     * @param sec_type @ref SecurityType of the profile to be saved
     * @param key password of the SSID, nullptr allowed if sec_type is
     *            @ref SEC_OPEN
     * @param priority connection priority when more than one of the saved
     *        networks is available, 0 == lowest priority
     * @return resulting index in the list of profiles, else -1 on error
     */
    int wlan_profile_add(const char *ssid, SecurityType sec_type,
                         const char *key, unsigned priority);

    /** Delete a saved WLAN profile.
     * @param ssid WLAN SSID of the profile to delete
     * @return 0 upon success, else -1 on error
     */
    int wlan_profile_del(const char *ssid);

    /** Delete a saved WLAN profile.
     * @param index index within saved profile list to remove, 0xFF removes all
     * @return 0 upon success, else -1 on error
     */
    int wlan_profile_del(int index);

    /** Get a saved WLAN profile by index.
     * @param index index within saved profile list to get
     * @param ssid 33 byte array that will return the ssid of the index
     * @param sec_type will return the security type of the index
     * @param priority will return the priority of the index
     * @return 0 upon success, else -1 on error
     */
    int wlan_profile_get(int index, char ssid[], SecurityType *sec_type,
                         uint32_t *priority);

    /** Test if there are any saved profiles.
     * @return true if there are no provides saved, else false
     */
    bool wlan_profile_test_none();

    /** Get a list of available networks.
     * @param entries returns a list of available network entries
     * @param count size of entry list in number of elements, max 20
     * @return number of valid network entries in the list
     */
    int wlan_network_list_get(WlanNetworkEntry *entries, size_t count);

    /** Get the device MAC address.
     * @param mac 6 byte array which will hold the resulting MAC address.
     */
    void wlan_mac(uint8_t mac[6]);

    /** Get the assigned IP address.
     * @return assigned IP address, else 0 if not assigned
     */
    uint32_t wlan_ip()
    {
        return ipAquired ? ipAddress : 0;
    }

    /** Get the SSID of the access point we are connected to.
     * @return SSID of the access point we are connected to
     */
    const char *wlan_ssid()
    {
        ssid[32] = '\0';
        return ssid;
    }

    /** Get the receive signal strength indicator.
     * @return receive signal strength
     */
    int wlan_rssi()
    {
        return rssi;
    }

    /** Executes the given function on the network thread. */
    void run_on_network_thread(std::function<void()> callback);

    /** This function handles WLAN events.  This is public only so that an
     * extern "C" method can call it.  DO NOT use directly.
     * @param context pointer to WLAN Event Info
     */
    void wlan_event_handler(void *context);

    /** This function handles network events such as IP acquisition, IP leased,
     * IP released etc.  This is public only so that an extern "C" method can
     * call it.  DO NOT use directly.
     * @param context Pointer indicating device acquired IP
     */
    void net_app_event_handler(void *context);

    /** This function handles socket events indication.  This is public only so
     * that an extern "C" method can call it.  DO NOT use directly.
     * @param context pointer to Socket Event Info
     */
    void sock_event_handler(void *context);

private:
    /** Set the CC32xx to its default state, including station mode.
     */
    void set_default_state();

    /** Thread that will manage the WLAN connection.
     * @param context context passed into the stack.
     */
    static void* wlan_task_entry(void *context)
    {
        instance()->wlan_task();
        return nullptr;
    }

    /** Thread that will manage the WLAN connection inside object context.
     */
    void wlan_task();

    /** Asynchronously wakeup the select call.
     * @param data -1 for no action, else socket descriptor if socket shall be
     *             closed.
     */
    void select_wakeup(int16_t data = -1);

    /** Add socket to the read fd set.
     * @param socket socket descriptor to add
     */
    void fd_set_read(int16_t socket);

    /** Add socket to the write fd set.
     * @param socket socket descriptor to add
     */
    void fd_set_write(int16_t socket);

    static CC32xxWiFi *instance_; /**< singleton instance pointer. */
    uint32_t ipAddress; /**< assigned IP adress */
    char ssid[33]; /**< SSID of AP we are connected to */

    /// List of callbacks to execute on the network thread.
    std::vector<std::function<void()> > callbacks_;
    /// Protects callbacks_ vector.
    OSMutex lock_;

    int wakeup; /**< loopback socket to wakeup select() */

    int16_t rssi; /**< receive signal strength indicator */

    unsigned connected        : 1; /**< AP connected state */
    unsigned connectionFailed : 1; /**< Connection attempt failed status */
    unsigned ipAquired        : 1; /**< IP address aquired state */
    unsigned ipLeased         : 1; /**< IP address leased to a client(AP mode)*/
    unsigned smartConfigStart : 1; /**< Smart config in progress */

    /** allow access to private members from CC32xxSocket */
    friend class CC32xxSocket;

    DISALLOW_COPY_AND_ASSIGN(CC32xxWiFi);
};

#endif /* _FREERTOS_DRIVERS_NET_C32XX_CC32XXWIFI_HXX_ */
