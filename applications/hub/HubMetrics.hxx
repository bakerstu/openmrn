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
 * \file HubMetrics.hxx
 *
 * Centralized metrics collection for hub application monitoring.
 *
 * @author Paul
 * @date 12 Aug 2026
 */

#ifndef _APPLICATIONS_HUB_HUBMETRICS_HXX_
#define _APPLICATIONS_HUB_HUBMETRICS_HXX_

#include <atomic>
#include <ctime>
#include <functional>
#include <string>

/// @brief Centralized metrics collection for hub monitoring
///
/// Thread-safe metrics collection using atomic operations.
/// All counters are lock-free and can be updated from any thread.
///
/// @todo Track packets transmitted and their byte counts, dropped/malformed
/// packet counts, socket/parse/timeout error counters, packet processing
/// latency, and queue depth. These are not currently tracked anywhere in the
/// hub application; add the necessary hooks in the relevant classes before
/// exposing them here.
class HubMetrics
{
public:
    /// Constructor
    HubMetrics()
        : connections_total_(0)
        , packets_rx_total_(0)
        , bytes_rx_total_(0)
        , start_time_(time(nullptr))
    {
    }

    // Connection metrics

    /// @brief Sets the callback used to query the number of currently active
    /// connections.
    /// @param provider Callback returning the current active connection
    /// count. The referenced object (if any) must outlive this HubMetrics
    /// instance.
    void set_num_clients_provider(std::function<unsigned()> provider)
    {
        num_clients_provider_ = std::move(provider);
    }

    /// @brief Record a new incoming connection. Intended to be used as (or
    /// from) a hub's "on connect" callback.
    void increment_connections()
    {
        connections_total_.fetch_add(1, std::memory_order_relaxed);
    }

    // Packet metrics

    /// @brief Record a received packet
    /// @param bytes Number of bytes in the packet
    void record_packet_rx(size_t bytes)
    {
        packets_rx_total_.fetch_add(1, std::memory_order_relaxed);
        bytes_rx_total_.fetch_add(bytes, std::memory_order_relaxed);
    }

    // Accessors (for JSON/Prometheus export)

    /// @return Current number of active connections
    uint64_t get_connections_active() const
    {
        return num_clients_provider_ ? num_clients_provider_() : 0;
    }

    /// @return Total connections since start
    uint64_t get_connections_total() const
    {
        return connections_total_.load(std::memory_order_relaxed);
    }

    /// @return Total packets received
    uint64_t get_packets_rx_total() const
    {
        return packets_rx_total_.load(std::memory_order_relaxed);
    }

    /// @return Total bytes received
    uint64_t get_bytes_rx_total() const
    {
        return bytes_rx_total_.load(std::memory_order_relaxed);
    }

    /// @return Uptime in seconds
    uint64_t get_uptime_seconds() const
    {
        return time(nullptr) - start_time_;
    }

    /// @return Start time (Unix timestamp)
    time_t get_start_time() const
    {
        return start_time_;
    }

    /// @brief Generate JSON representation of metrics
    /// @return JSON string with all metrics
    std::string to_json() const;

    /// @brief Print summary to stderr
    void print_summary() const;

private:
    // Connection metrics
    std::atomic<uint64_t> connections_total_;
    /// Callback used to query the number of currently active connections.
    std::function<unsigned()> num_clients_provider_;

    // Packet metrics
    std::atomic<uint64_t> packets_rx_total_;
    std::atomic<uint64_t> bytes_rx_total_;

    // System metrics
    time_t start_time_;
};

#endif // _APPLICATIONS_HUB_HUBMETRICS_HXX_
