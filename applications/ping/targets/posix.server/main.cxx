#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include <memory>

#include "os/os.h"
#include "utils/socket_listener.hxx"

#include "protocol.hxx"
#include "utils.hxx"

void *socket_receive_thread(void *arg)
{
    std::unique_ptr<ThreadArg> a((ThreadArg *)arg);
    Request r;
    long long first_req;
    if (!rw_repeated(a->fd, &r, sizeof(r), &first_req, read, "read"))
    {
        return nullptr;
    }
    long long all_header = os_get_time_monotonic();
    int maxlen = std::max(r.payload_length, r.response_length);
    std::unique_ptr<uint8_t[]> payload(new uint8_t[maxlen]);
    long long first_body;
    memset(payload.get(), 0x5A, maxlen);
    if (!rw_repeated(
            a->fd, payload.get(), r.payload_length, &first_body, read, "read"))
    {
        return nullptr;
    }
    long long all_payload_arrived = os_get_time_monotonic();

    // Send response
    long long first_response;
    if (!rw_repeated(
            a->fd, payload.get(), r.response_length, &first_response, write, "write"))
    {
        return nullptr;
    }
    long long all_response = os_get_time_monotonic();

    LatencyResponse l;
    long long first_latency;
    if (!rw_repeated(a->fd, &l, sizeof(l), &first_latency, read, "read"))
    {
        return nullptr;
    }
    long long all_latency = os_get_time_monotonic();
    
    printstat("first hdr->all hdr", first_req, all_header);
    printstat("all hdr->first body", all_header, first_body);
    printstat("all hdr->first body", first_body, all_payload_arrived);
    printf("\n");
    printstat("first hdr->all request", first_req, all_payload_arrived);

    printstat("all request->first_resp", all_payload_arrived, first_response);
    printstat("first resp->all resp", first_response, all_response);

    printstat("all resp->first latency", all_response, first_latency);

    printstat("first latency->all_latency", first_latency, all_latency);

    printf("\n");
    printf("Remote latency: %d usec", l.latency_usec);
    
    ::close(a->fd);
    return nullptr;
}

void on_new_connection(int fd)
{
    ThreadArg* a = new ThreadArg;
    a->fd = fd;
    os_thread_t thread;
    os_thread_create(&thread, "recvthread", 0, 1500, &socket_receive_thread, a);
}

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    SocketListener listener(30268, &on_new_connection);
    while (true)
        ;
    return 0;
}
