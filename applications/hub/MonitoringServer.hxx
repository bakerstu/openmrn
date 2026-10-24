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
 * \file MonitoringServer.hxx
 *
 * Lightweight HTTP server for monitoring endpoints.
 *
 * @author Paul
 * @date 12 Aug 2026
 */

#ifndef _APPLICATIONS_HUB_MONITORINGSERVER_HXX_
#define _APPLICATIONS_HUB_MONITORINGSERVER_HXX_

#include <pthread.h>
#include <atomic>
#include <string>

class HubMetrics;

/// @brief Lightweight HTTP server for monitoring endpoints
///
/// Provides /health and /status endpoints for operational monitoring.
/// Runs in a separate thread to avoid blocking packet processing.
class MonitoringServer
{
public:
    /// @brief Constructor
    /// @param metrics Pointer to metrics object to expose
    /// @param port TCP port to listen on (default: 8080)
    /// @param bind_address Address to bind to (default: "127.0.0.1" localhost only)
    MonitoringServer(HubMetrics *metrics, int port = 8080,
                     const char *bind_address = "127.0.0.1");

    /// Destructor
    ~MonitoringServer();

    /// @brief Start the monitoring server
    /// @return true if server started successfully
    bool start();

    /// @brief Stop the monitoring server
    void stop();

    /// @brief Check if server is running
    /// @return true if server is running
    bool is_running() const
    {
        return running_.load(std::memory_order_relaxed);
    }

    /// @brief Check if server is ready to accept connections
    /// @return true if server socket is bound and listening
    bool is_ready() const
    {
        return ready_.load(std::memory_order_relaxed);
    }

private:
    /// Server thread entry point
    static void *server_thread_entry(void *arg);

    /// Run the server loop
    void run_server();

    /// Handle a single HTTP request
    /// @param client_fd Client socket file descriptor
    void handle_request(int client_fd);

    /// Serve the /health endpoint
    /// @param client_fd Client socket file descriptor
    void serve_health(int client_fd);

    /// Serve the /status endpoint
    /// @param client_fd Client socket file descriptor
    void serve_status(int client_fd);

    /// Send HTTP response
    /// @param client_fd Client socket
    /// @param status_code HTTP status code
    /// @param content_type Content-Type header value
    /// @param body Response body
    void send_response(int client_fd, int status_code,
                      const char *content_type, const std::string &body);

    HubMetrics *metrics_;        ///< Metrics object to expose
    int port_;                   ///< TCP port to listen on
    std::string bind_address_;   ///< Address to bind to
    int server_fd_;              ///< Server socket file descriptor
    pthread_t thread_;           ///< Server thread handle
    std::atomic<bool> running_;  ///< Server running flag
    std::atomic<bool> ready_;    ///< Server ready flag
    std::atomic<bool> stop_requested_; ///< Stop request flag
};

#endif // _APPLICATIONS_HUB_MONITORINGSERVER_HXX_
