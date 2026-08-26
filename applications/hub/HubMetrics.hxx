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
#include <string>
#include "utils/Stats.hxx"

/// @brief Centralized metrics collection for hub monitoring
///
/// Thread-safe metrics collection using atomic operations.
/// All counters are lock-free and can be updated from any thread.
class HubMetrics
{
public:
    /// Constructor
    HubMetrics()
        : connections_active_(0)
        , connections_total_(0)
        , connections_failed_(0)
        , packets_rx_total_(0)
        , packets_tx_total_(0)
        , packets_dropped_(0)
        , packets_malformed_(0)
        , bytes_rx_total_(0)
        , bytes_tx_total_(0)
        , errors_socket_(0)
        , errors_parse_(0)
        , errors_timeout_(0)
        , queue_depth_(0)
        , start_time_(time(nullptr))
    {
    }

    // Connection metrics

    /// @brief Increment active connection count
    void increment_connections()
    {
        connections_active_.fetch_add(1, std::memory_order_relaxed);
        connections_total_.fetch_add(1, std::memory_order_relaxed);
    }

    /// @brief Decrement active connection count
    void decrement_connections()
    {
        connections_active_.fetch_sub(1, std::memory_order_relaxed);
    }

    /// @brief Record a failed connection attempt
    void record_connection_failed()
    {
        connections_failed_.fetch_add(1, std::memory_order_relaxed);
    }

    // Packet metrics

    /// @brief Record a received packet
    /// @param bytes Number of bytes in the packet
    void record_packet_rx(size_t bytes = 0)
    {
        packets_rx_total_.fetch_add(1, std::memory_order_relaxed);
        if (bytes > 0)
        {
            bytes_rx_total_.fetch_add(bytes, std::memory_order_relaxed);
        }
    }

    /// @brief Record a transmitted packet
    /// @param bytes Number of bytes in the packet
    void record_packet_tx(size_t bytes = 0)
    {
        packets_tx_total_.fetch_add(1, std::memory_order_relaxed);
        if (bytes > 0)
        {
            bytes_tx_total_.fetch_add(bytes, std::memory_order_relaxed);
        }
    }

    /// @brief Record a dropped packet
    void record_packet_dropped()
    {
        packets_dropped_.fetch_add(1, std::memory_order_relaxed);
    }

    /// @brief Record a malformed packet
    void record_packet_malformed()
    {
        packets_malformed_.fetch_add(1, std::memory_order_relaxed);
    }

    // Error metrics

    /// @brief Record a socket error
    void record_socket_error()
    {
        errors_socket_.fetch_add(1, std::memory_order_relaxed);
    }

    /// @brief Record a parse error
    void record_parse_error()
    {
        errors_parse_.fetch_add(1, std::memory_order_relaxed);
    }

    /// @brief Record a timeout error
    void record_timeout_error()
    {
        errors_timeout_.fetch_add(1, std::memory_order_relaxed);
    }

    // Performance metrics

    /// @brief Record packet processing latency
    /// @param latency_us Latency in microseconds
    void record_latency(int32_t latency_us)
    {
        packet_latency_us_.add(latency_us);
    }

    /// @brief Set current queue depth
    /// @param depth Current queue depth
    void set_queue_depth(uint32_t depth)
    {
        queue_depth_.store(depth, std::memory_order_relaxed);
    }

    // Accessors (for JSON/Prometheus export)

    /// @return Current number of active connections
    uint64_t get_connections_active() const
    {
        return connections_active_.load(std::memory_order_relaxed);
    }

    /// @return Total connections since start
    uint64_t get_connections_total() const
    {
        return connections_total_.load(std::memory_order_relaxed);
    }

    /// @return Number of failed connections
    uint64_t get_connections_failed() const
    {
        return connections_failed_.load(std::memory_order_relaxed);
    }

    /// @return Total packets received
    uint64_t get_packets_rx_total() const
    {
        return packets_rx_total_.load(std::memory_order_relaxed);
    }

    /// @return Total packets transmitted
    uint64_t get_packets_tx_total() const
    {
        return packets_tx_total_.load(std::memory_order_relaxed);
    }

    /// @return Total packets dropped
    uint64_t get_packets_dropped() const
    {
        return packets_dropped_.load(std::memory_order_relaxed);
    }

    /// @return Total malformed packets
    uint64_t get_packets_malformed() const
    {
        return packets_malformed_.load(std::memory_order_relaxed);
    }

    /// @return Total bytes received
    uint64_t get_bytes_rx_total() const
    {
        return bytes_rx_total_.load(std::memory_order_relaxed);
    }

    /// @return Total bytes transmitted
    uint64_t get_bytes_tx_total() const
    {
        return bytes_tx_total_.load(std::memory_order_relaxed);
    }

    /// @return Number of socket errors
    uint64_t get_errors_socket() const
    {
        return errors_socket_.load(std::memory_order_relaxed);
    }

    /// @return Number of parse errors
    uint64_t get_errors_parse() const
    {
        return errors_parse_.load(std::memory_order_relaxed);
    }

    /// @return Number of timeout errors
    uint64_t get_errors_timeout() const
    {
        return errors_timeout_.load(std::memory_order_relaxed);
    }

    /// @return Current queue depth
    uint32_t get_queue_depth() const
    {
        return queue_depth_.load(std::memory_order_relaxed);
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

    /// @return Reference to latency statistics
    const Stats& get_latency_stats() const
    {
        return packet_latency_us_;
    }

    /// @brief Generate JSON representation of metrics
    /// @return JSON string with all metrics
    std::string to_json() const;

    /// @brief Print summary to stderr
    void print_summary() const;

private:
    // Connection metrics
    std::atomic<uint64_t> connections_active_;
    std::atomic<uint64_t> connections_total_;
    std::atomic<uint64_t> connections_failed_;

    // Packet metrics
    std::atomic<uint64_t> packets_rx_total_;
    std::atomic<uint64_t> packets_tx_total_;
    std::atomic<uint64_t> packets_dropped_;
    std::atomic<uint64_t> packets_malformed_;
    std::atomic<uint64_t> bytes_rx_total_;
    std::atomic<uint64_t> bytes_tx_total_;

    // Error metrics
    std::atomic<uint64_t> errors_socket_;
    std::atomic<uint64_t> errors_parse_;
    std::atomic<uint64_t> errors_timeout_;

    // Performance metrics
    Stats packet_latency_us_;
    std::atomic<uint32_t> queue_depth_;

    // System metrics
    time_t start_time_;
};

#endif // _APPLICATIONS_HUB_HUBMETRICS_HXX_
