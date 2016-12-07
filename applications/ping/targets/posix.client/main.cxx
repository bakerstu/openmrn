#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include <memory>

#include "os/os.h"
#include "utils/socket_listener.hxx"

#include "protocol.hxx"
#include "utils.hxx"

char *host = nullptr;
int port = DEFAULT_PORT;
int req_bytes = 20;
int resp_bytes = 20;
int sleep_msec = 1000;

void run_client(int fd)
{
    while (true)
    {
        Request r;
        r.payload_length = req_bytes;
        r.response_length = resp_bytes;
        long long before_send = os_get_time_monotonic();
        int maxlen = std::max(r.payload_length + sizeof(r), (size_t)r.response_length);
        std::unique_ptr<uint8_t[]> payload(new uint8_t[maxlen]);
        memset(payload.get(), 0xAA, maxlen);
        memcpy(payload.get(), &r, sizeof(r));
        long long first_write;
        if (!rw_repeated(fd, payload.get(), r.payload_length + sizeof(r),
                &first_write, write, "write"))
        {
            exit(1);
        }
        long long write_done = os_get_time_monotonic();

        long long first_response;
        if (!rw_repeated(fd, payload.get(), r.response_length, &first_response,
                read, "read"))
        {
            exit(1);
        }
        long long all_response = os_get_time_monotonic();

        LatencyResponse l;
        l.latency_usec = (all_response - before_send) / 1000;

        if (!rw_repeated(fd, &l, sizeof(l), nullptr, write, "write"))
        {
            exit(1);
        }

        printstat("before hdr->first_write", before_send, first_write);
        printstat("first_write->write_done", first_write, write_done);

        printf("\n");
        printstat("all write", before_send, write_done);
        printf("\n");
        printstat("write_done->first_resp", write_done, first_response);
        printstat("first_resp->all_resp", first_response, all_response);

        printf("\n");
        printstat("e2e", before_send, all_response);
        usleep(sleep_msec * 1000);
    }
}

void usage(const char *e)
{
    fprintf(stderr, "Usage: %s [-u host] [-p port] [-w wait_msec] [-r resp_bytes] [-q req_bytes]\n\n", e);
    fprintf(stderr, "TCP ping client.\n\nArguments:\n");
    fprintf(stderr, "\t-p port     specifies the port number to connect to, "
                    "default is 30268.\n");
    fprintf(stderr, "\t-u host   is the host name for the ping server.\n");
    exit(1);
}

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hu:p:q:r:w:")) >= 0)
    {
        switch (opt)
        {
            case 'h':
                usage(argv[0]);
                break;
            case 'p':
                port = atoi(optarg);
                break;
            case 'u':
                host = optarg;
                break;
            case 'q':
                req_bytes = atoi(optarg);
                break;
            case 'r':
                resp_bytes = atoi(optarg);
                break;
            case 'w':
                sleep_msec = atoi(optarg);
                break;
            default:
                fprintf(stderr, "Unknown option %c\n", opt);
                usage(argv[0]);
        }
    }
    if (!host)
        usage(argv[0]);
}

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    parse_args(argc, argv);
    long long before_connect = os_get_time_monotonic();
    int fd = ConnectSocket(host, port);
    long long after_connect = os_get_time_monotonic();
    if (fd < 0)
    {
        fprintf(stderr, "failed to connect");
        exit(1);
    }
    printstat("connect", before_connect, after_connect);
    usleep(500000);
    run_client(fd);
    ::close(fd);
    return 0;
}
