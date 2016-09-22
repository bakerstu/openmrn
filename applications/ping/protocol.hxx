#ifndef _PING_PROTOCOL_HXX_
#define _PING_PROTOCOL_HXX_

#define DEFAULT_PORT 30263

// Protocol: Receive first four bytes: packet length.
struct Request
{
    /// How many bytes of payload we should be sending. (little-endian)
    int payload_length;
    /// How many bytes of response payload we want back. (little-endian)
    int response_length;
};

struct LatencyResponse
{
    int latency_usec;
};


#endif // _PING_PROTOCOL_HXX_
