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
 * \file MonitoringServer.cxx
 *
 * Implementation of lightweight HTTP monitoring server.
 *
 * @author Paul
 * @date 12 Aug 2026
 */

#include "MonitoringServer.hxx"
#include "HubMetrics.hxx"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

MonitoringServer::MonitoringServer(HubMetrics *metrics, int port,
                                   const char *bind_address)
    : metrics_(metrics)
    , port_(port)
    , bind_address_(bind_address)
    , server_fd_(-1)
    , thread_(0)
    , running_(false)
    , ready_(false)
    , stop_requested_(false)
{
}

MonitoringServer::~MonitoringServer()
{
    stop();
}

bool MonitoringServer::start()
{
    if (running_.load(std::memory_order_relaxed))
    {
        return true; // Already running
    }

    // Create server socket
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0)
    {
        fprintf(stderr, "MonitoringServer: Failed to create socket: %s\n",
                strerror(errno));
        return false;
    }

    // Set SO_REUSEADDR to avoid "Address already in use" errors
    int opt = 1;
    if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
    {
        fprintf(stderr, "MonitoringServer: Failed to set SO_REUSEADDR: %s\n",
                strerror(errno));
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    // Bind to address and port
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    if (inet_pton(AF_INET, bind_address_.c_str(), &addr.sin_addr) <= 0)
    {
        fprintf(stderr, "MonitoringServer: Invalid bind address: %s\n",
                bind_address_.c_str());
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    if (bind(server_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        fprintf(stderr, "MonitoringServer: Failed to bind to %s:%d: %s\n",
                bind_address_.c_str(), port_, strerror(errno));
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    // Listen for connections
    if (listen(server_fd_, 5) < 0)
    {
        fprintf(stderr, "MonitoringServer: Failed to listen: %s\n",
                strerror(errno));
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    // Create server thread
    running_.store(true, std::memory_order_relaxed);
    stop_requested_.store(false, std::memory_order_relaxed);

    int result = pthread_create(&thread_, nullptr, server_thread_entry, this);
    if (result != 0)
    {
        fprintf(stderr, "MonitoringServer: Failed to create thread: %s\n",
                strerror(result));
        running_.store(false, std::memory_order_relaxed);
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    pthread_detach(thread_);

    fprintf(stderr, "MonitoringServer: Listening on http://%s:%d\n",
            bind_address_.c_str(), port_);

    return true;
}

void MonitoringServer::stop()
{
    if (!running_.load(std::memory_order_relaxed))
    {
        return;
    }

    stop_requested_.store(true, std::memory_order_relaxed);
    running_.store(false, std::memory_order_relaxed);

    if (server_fd_ >= 0)
    {
        close(server_fd_);
        server_fd_ = -1;
    }

    // Give the thread a moment to exit
    usleep(100000); // 100ms
}

void *MonitoringServer::server_thread_entry(void *arg)
{
    MonitoringServer *server = static_cast<MonitoringServer *>(arg);
    server->run_server();
    return nullptr;
}

void MonitoringServer::run_server()
{
    ready_.store(true, std::memory_order_relaxed);

    while (!stop_requested_.load(std::memory_order_relaxed))
    {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);

        // Set a timeout on accept so we can check stop_requested periodically
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        setsockopt(server_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        int client_fd = accept(server_fd_, (struct sockaddr *)&client_addr,
                              &client_len);

        if (client_fd < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                continue; // Timeout, check stop_requested
            }
            if (stop_requested_.load(std::memory_order_relaxed))
            {
                break; // Shutting down
            }
            // Other error
            continue;
        }

        // Handle request (inline for simplicity)
        handle_request(client_fd);
        close(client_fd);
    }

    ready_.store(false, std::memory_order_relaxed);
}

void MonitoringServer::handle_request(int client_fd)
{
    char buffer[1024];
    ssize_t n = read(client_fd, buffer, sizeof(buffer) - 1);
    if (n <= 0)
    {
        return;
    }
    buffer[n] = '\0';

    // Parse HTTP request line
    char method[16], path[256], version[16];
    if (sscanf(buffer, "%15s %255s %15s", method, path, version) != 3)
    {
        send_response(client_fd, 400, "text/plain", "Bad Request");
        return;
    }

    // Route to appropriate handler
    if (strcmp(path, "/health") == 0)
    {
        serve_health(client_fd);
    }
    else if (strcmp(path, "/status") == 0)
    {
        serve_status(client_fd);
    }
    else
    {
        send_response(client_fd, 404, "text/plain", "Not Found");
    }
}

void MonitoringServer::serve_health(int client_fd)
{
    send_response(client_fd, 200, "text/plain", "OK\n");
}

void MonitoringServer::serve_status(int client_fd)
{
    std::string json = metrics_->to_json();
    send_response(client_fd, 200, "application/json", json);
}

void MonitoringServer::send_response(int client_fd, int status_code,
                                     const char *content_type,
                                     const std::string &body)
{
    const char *status_text;
    switch (status_code)
    {
        case 200: status_text = "OK"; break;
        case 400: status_text = "Bad Request"; break;
        case 404: status_text = "Not Found"; break;
        case 500: status_text = "Internal Server Error"; break;
        default: status_text = "Unknown"; break;
    }

    char header[512];
    snprintf(header, sizeof(header),
             "HTTP/1.0 %d %s\r\n"
             "Content-Type: %s\r\n"
             "Content-Length: %zu\r\n"
             "Connection: close\r\n"
             "\r\n",
             status_code, status_text, content_type, body.length());

    write(client_fd, header, strlen(header));
    write(client_fd, body.c_str(), body.length());
}
