
#include "utils/HubDeviceSelect.hxx"

#include "nmranet_config.h"

/// @return the number of packets to limit read input if we are throttling.
int hubdevice_incoming_packet_limit()
{
    return config_gridconnect_port_max_incoming_packets();
}
