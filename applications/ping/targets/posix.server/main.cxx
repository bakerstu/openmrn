#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h> /* tc* functions */
#include <stdio.h>

#include <memory>

#include "os/os.h"
#include "utils/socket_listener.hxx"

#include "protocol.hxx"
#include "utils.hxx"
#include "histogram.hxx"

bool display_detailed = false;
const char *device = nullptr;
int port = DEFAULT_PORT; // 30263

volatile bool is_alive = false;
volatile long long last_ping_received;
volatile int num_pings = 0;

Histogram client_latency_histo;
Histogram server_latency_histo;

void *socket_receive_thread(void *arg)
{
    std::unique_ptr<ThreadArg> a((ThreadArg *)arg);
    struct Deleter
    {
        ~Deleter()
        {
            is_alive = false;
        }
    } guard;
    while (true)
    {
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
        long long before_body = os_get_time_monotonic();
        if (!rw_repeated(a->fd, payload.get(), r.payload_length, &first_body,
                read, "read"))
        {
            return nullptr;
        }
        long long all_payload_arrived = os_get_time_monotonic();

        // Send response
        long long first_response;
        if (!rw_repeated(a->fd, payload.get(), r.response_length,
                &first_response, write, "write"))
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

        last_ping_received = all_latency;
        ++num_pings;
        int server_latency_usec = int(first_latency - all_response) / 1000;
        client_latency_histo.add(l.latency_usec);
        server_latency_histo.add(server_latency_usec);
        if (display_detailed)
        {
            printstat("first hdr->all hdr", first_req, all_header);
            printstat("all hdr->before body", all_header, before_body);
            printstat("before_body->first body", before_body, first_body);
            printstat(
                "first body->all payload", first_body, all_payload_arrived);
            printf("\n");
            printstat("first hdr->all request", first_req, all_payload_arrived);

            printstat(
                "all request->first_resp", all_payload_arrived, first_response);
            printstat("first resp->all resp", first_response, all_response);

            printstat("all resp->first latency", all_response, first_latency);

            printstat("first latency->all_latency", first_latency, all_latency);

            printf("\n");
            printf("Remote latency: %d usec\n", l.latency_usec);
        }
        else
        {
            printf("Server latency seen by client: %6d usec; client latency "
                   "seen by server: %6d usec\n",
                l.latency_usec, server_latency_usec);
        }
    }
    ::close(a->fd);
    return nullptr;
}

void on_new_connection(int fd)
{
    ThreadArg *a = new ThreadArg;
    a->fd = fd;
    os_thread_t thread;
    os_thread_create(&thread, "recvthread", 0, 1500, &socket_receive_thread, a);
}

void usage(const char *e)
{
    fprintf(stderr, "Usage: %s [-d device] [-p port] [-w wait_msec]\n\n", e);
    fprintf(stderr, "TCP ping client.\n\nArguments:\n");
    fprintf(stderr, "\t-p port     specifies the port number to listen on, "
                    "default is 30268.\n");
    fprintf(stderr, "\t-d device   If specified, will open the given serial "
                    "port and wait for requests there too.\n");
    exit(1);
}

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hd:p:v")) >= 0)
    {
        switch (opt)
        {
            case 'h':
                usage(argv[0]);
                break;
            case 'p':
                port = atoi(optarg);
                break;
            case 'd':
                device = optarg;
                break;
            case 'v':
                display_detailed = true;
                break;
            default:
                fprintf(stderr, "Unknown option %c\n", opt);
                usage(argv[0]);
        }
    }
}

void sleep_or_print_histo() {
    if (!num_pings) {
        sleep(1);
        return;
    }
    if (os_get_time_monotonic() - last_ping_received > MSEC_TO_NSEC(1500)) {
        printf("Server latency seen by client ");
        client_latency_histo.print();
        printf("\n");
        
        printf("Client latency seen by server ");
        server_latency_histo.print();
        printf("\n");
        num_pings = 0;
    }
    sleep(1);
}

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    parse_args(argc, argv);
    while (device)
    {
        int fd = ::open(device, O_RDWR);
        if (fd >= 0)
        {
            // Sets up the terminal in raw mode. Otherwise linux might echo
            // characters coming in from the device and that will make
            // packets go back to where they came from.
            HASSERT(!tcflush(fd, TCIOFLUSH));
            struct termios settings;
            HASSERT(!tcgetattr(fd, &settings));
            cfmakeraw(&settings);
            cfsetspeed(&settings, B115200);
            HASSERT(!tcsetattr(fd, TCSANOW, &settings));
            LOG(INFO, "Opened device %s.\n", device);
            is_alive = true;
            on_new_connection(fd);
            while (is_alive)
            {
                sleep_or_print_histo();
            }
        }
        else
        {
            LOG_ERROR("Could not open device %s\n", device);
            sleep(1);
        }
    }
    // else: no device specified.
    SocketListener listener(port, &on_new_connection);
    while (true)
    {
        sleep_or_print_histo();
    }
    return 0;
}
