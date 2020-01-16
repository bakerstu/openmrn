#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include <memory>

#include "os/os.h"
#include "utils/socket_listener.hxx"
#include "freertos_drivers/common/WifiDefs.hxx"
#include "freertos_drivers/net_cc32xx/CC32xxWiFi.hxx"

#include "protocol.hxx"
#include "utils.hxx"

#include "hardware.hxx"

int req_bytes = 20;
int resp_bytes = 20;

#define printstat(x, bef, after) testfn(bef, after)

void run_client(int fd)
{
    Request r;
    r.payload_length = req_bytes;
    r.response_length = resp_bytes;
    long long before_send = os_get_time_monotonic();
    long long first_hdr;
    if (!rw_repeated(fd, &r, sizeof(r), &first_hdr, write, "write"))
    {
        diewith(ERR_WRITE_1);
    }
    long long after_hdr = os_get_time_monotonic();
    int maxlen = std::max(r.payload_length, r.response_length);
    std::unique_ptr<uint8_t[]> payload(new uint8_t[maxlen]);
    memset(payload.get(), 0xAA, maxlen);
    long long first_write;
    if (!rw_repeated(
            fd, payload.get(), r.payload_length, &first_write, write, "write"))
    {
        diewith(ERR_WRITE_2);
    }
    long long write_done = os_get_time_monotonic();

    long long first_response;
    if (!rw_repeated(fd, payload.get(), r.response_length, &first_response,
            read, "read"))
    {
        diewith(ERR_READ_1);
    }
    long long all_response = os_get_time_monotonic();

    LatencyResponse l;
    l.latency_usec = (all_response - before_send) / 1000;

    if (!rw_repeated(fd, &l, sizeof(l), nullptr, write, "write"))
    {
        diewith(ERR_WRITE_3);
    }

    printstat("before hdr->first hdr", before_send, first_hdr);
    printstat("first hdr->after_hdr", first_hdr, after_hdr);
    printstat("after_hdr->first_write", after_hdr, first_write);
    printstat("first_write->write_done", first_write, write_done);

    // printf("\n");
    printstat("all write", before_send, write_done);
    // printf("\n");
    printstat("write_done->first_resp", write_done, first_response);
    printstat("first_resp->all_resp", first_response, all_response);

    // printf("\n");
    printstat("e2e", before_send, all_response);
}

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    auto *wifi = CC32xxWiFi::instance();
    wifi->wlan_connect(WIFI_SSID, WIFI_PASS,
        strlen(WIFI_PASS) > 0 ? CC32xxWiFi::SEC_WPA2 : CC32xxWiFi::SEC_OPEN);

    while (!wifi->wlan_ready())
    {
        wifi->connecting_update_blinker();
        usleep(10000);
    }
    resetblink(WIFI_BLINK_CONNECTING);

    long long before_connect = os_get_time_monotonic();
    int fd = ConnectSocket(WIFI_HUB_HOSTNAME, WIFI_HUB_PORT);
    long long after_connect = os_get_time_monotonic();
    resetblink(0);
    if (fd < 0)
    {
        diewith(WIFI_BLINK_FAILED);
    }
    printstat("connect", before_connect, after_connect);
    while (true)
    {
        do
        {
            usleep(200000);
        } while (!SW3_Pin::get());
        run_client(fd);
    }
    ::close(fd);
    return 0;
}
