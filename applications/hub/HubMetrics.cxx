/** \copyright
 * Copyright (c) 2026, Paul
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
 * \file HubMetrics.cxx
 *
 * Implementation of metrics collection for hub application.
 *
 * @author Paul
 * @date 12 Aug 2026
 */

#include "HubMetrics.hxx"
#include <sstream>
#include <iomanip>
#include <stdio.h>

std::string HubMetrics::to_json() const
{
    std::ostringstream json;
    json << std::fixed << std::setprecision(2);
    
    uint64_t uptime = get_uptime_seconds();
    uint64_t rx_total = get_packets_rx_total();
    uint64_t tx_total = get_packets_tx_total();
    uint64_t errors_total = get_errors_socket() + get_errors_parse() + get_errors_timeout();
    
    // Calculate rates (packets per second)
    double rate_rx = uptime > 0 ? (double)rx_total / uptime : 0.0;
    double rate_tx = uptime > 0 ? (double)tx_total / uptime : 0.0;
    
    json << "{\n";
    json << "  \"status\": \"healthy\",\n";
    json << "  \"uptime_seconds\": " << uptime << ",\n";
    
    // Format start time as ISO 8601
    char time_buf[64];
    struct tm tm_info;
    localtime_r(&start_time_, &tm_info);
    strftime(time_buf, sizeof(time_buf), "%Y-%m-%dT%H:%M:%S%z", &tm_info);
    json << "  \"start_time\": \"" << time_buf << "\",\n";
    
    // Connections
    json << "  \"connections\": {\n";
    json << "    \"active\": " << get_connections_active() << ",\n";
    json << "    \"total\": " << get_connections_total() << ",\n";
    json << "    \"failed\": " << get_connections_failed() << "\n";
    json << "  },\n";
    
    // Packets
    json << "  \"packets\": {\n";
    json << "    \"received\": " << rx_total << ",\n";
    json << "    \"sent\": " << tx_total << ",\n";
    json << "    \"dropped\": " << get_packets_dropped() << ",\n";
    json << "    \"malformed\": " << get_packets_malformed() << ",\n";
    json << "    \"rate_rx_per_sec\": " << rate_rx << ",\n";
    json << "    \"rate_tx_per_sec\": " << rate_tx << "\n";
    json << "  },\n";
    
    // Bytes
    json << "  \"bytes\": {\n";
    json << "    \"received\": " << get_bytes_rx_total() << ",\n";
    json << "    \"sent\": " << get_bytes_tx_total() << "\n";
    json << "  },\n";
    
    // Performance
    // Note: Stats class doesn't expose count/max, so we just output what we can
    // The values will be 0 if no samples have been added yet
    json << "  \"performance\": {\n";
    json << "    \"queue_depth\": " << get_queue_depth() << "\n";
    json << "  },\n";
    
    // Errors
    json << "  \"errors\": {\n";
    json << "    \"socket\": " << get_errors_socket() << ",\n";
    json << "    \"parse\": " << get_errors_parse() << ",\n";
    json << "    \"timeout\": " << get_errors_timeout() << ",\n";
    json << "    \"total\": " << errors_total << "\n";
    json << "  }\n";
    
    json << "}\n";
    
    return json.str();
}

void HubMetrics::print_summary() const
{
    uint64_t uptime = get_uptime_seconds();
    uint64_t uptime_hours = uptime / 3600;
    uint64_t uptime_minutes = (uptime % 3600) / 60;
    uint64_t uptime_seconds = uptime % 60;
    
    uint64_t rx_total = get_packets_rx_total();
    uint64_t tx_total = get_packets_tx_total();
    uint64_t errors_total = get_errors_socket() + get_errors_parse() + get_errors_timeout();
    
    double rate = uptime > 0 ? (double)rx_total / uptime : 0.0;
    
    fprintf(stderr, "[HUB STATS] Uptime: %luh %lum %lus | "
            "Connections: %lu active, %lu total | "
            "Packets: %lu rx, %lu tx (%.1f/sec) | "
            "Errors: %lu total\n",
            (unsigned long)uptime_hours,
            (unsigned long)uptime_minutes,
            (unsigned long)uptime_seconds,
            (unsigned long)get_connections_active(),
            (unsigned long)get_connections_total(),
            (unsigned long)rx_total,
            (unsigned long)tx_total,
            rate,
            (unsigned long)errors_total);
}
