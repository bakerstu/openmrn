/** \copyright
 * Copyright (c) 2019, Mike Dunston
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
 * \file Esp32WiFiClientAdapter.hxx
 *
 * ESP32 adapter code using the WiFiClient provided by the WiFiServer code
 * for interfacing with the OpenMRN stack.
 *
 * @author Mike Dunston
 * @date 13 January 2019
 */

#ifndef _FREERTOS_DRIVERS_ESP32_ESP32WIFI_HXX_
#define _FREERTOS_DRIVERS_ESP32_ESP32WIFI_HXX_

#include <Arduino.h>
#include <WiFi.h>
#include "utils/logging.h"

class Esp32WiFiClientAdapter
{
public:
    /// Constructor.
    ///
    /// @param client is the client returned from the ESP32 WiFiServer.
    Esp32WiFiClientAdapter(WiFiClient client)
        : client_(client), remoteIP_(client.remoteIP()),
        remotePort_(client.remotePort())
    {
        client_.setNoDelay(true);
        // TODO: should we set the client RX/TX timeout?
    }

    /// This is how many bytes we return as writeable when select says the
    /// socket is write active.
    static constexpr unsigned WRITE_PACKET_SIZE = 512;

    /// @return the capacity of the write buffer for the underlying WiFiClient.
    size_t availableForWrite()
    {
        if (!connected())
        {
            LOG(VERBOSE, "EOF detected for fd: %d, %d (%s)", client_.fd(),
                errno, strerror(errno));
            return 0;
        }
        int fd = client_.fd();
        fd_set set;
        struct timeval tv;
        FD_ZERO(&set);    // empties the set
        FD_SET(fd, &set); // adds FD to the set
        tv.tv_sec = 0;
        tv.tv_usec = 0;

        if (select(fd + 1, NULL, &set, NULL, &tv) < 0)
        {
            return 0;
        }

        if (FD_ISSET(fd, &set))
        {
            return WRITE_PACKET_SIZE;
        }
        else
        {
            return 0;
        }
    }

    /// Writes a byte stream to the underlying WiFiClient.
    ///
    /// @param buffer byte stream to be transmitted.
    /// @param len length of byte stream to be transmitted.
    size_t write(const char *buffer, size_t len)
    {
        size_t bytesWritten = 0;
        if (connected())
        {
            bytesWritten = client_.write(buffer, len);
            if(!bytesWritten)
            {
                LOG(VERBOSE, "EOF detected for fd: %d, %d (%s)", client_.fd(),
                    errno, strerror(errno));
            }
        }
        return bytesWritten;
    }

    /// @return the number of bytes available to read from the underlying WiFiClient.
    size_t available()
    {
        if (connected())
        {
            return client_.available();
        }
        LOG(VERBOSE, "EOF detected for fd: %d, %d (%s)", client_.fd(), errno,
            strerror(errno));
        return 0;
    }

    /// Reads a byte stream from the underlying WiFiClient.
    ///
    /// @param buffer buffer to read into.
    /// @param len size of the buffer to read into.
    size_t read(const char *buffer, size_t len)
    {
        size_t bytesRead = 0;
        if (connected())
        {
            bytesRead = client_.read((uint8_t *)buffer, len);
        }
        if(!bytesRead)
        {
            LOG(VERBOSE, "EOF detected for fd: %d, %d (%s)", client_.fd(),
                errno, strerror(errno));
        }
        return bytesRead;
    }

    /// @return true if the underlying WiFiClient is still connected.
    bool connected()
    {
        // Ensure we do not return true after stop() has been called since
        // the underlying client_.connected() call may still return true since
        // the fd has not been reset by calling client_.stop() but it has
        // cleaned up resources already.
        if(remotePort_ == 0 || remoteIP_ == INADDR_NONE)
        {
            return false;
        }
        return client_.connected();
    }

    /// @return remote IP address from the underlying WiFiClient.
    IPAddress remoteIP() const
    {
        return remoteIP_;
    }

    /// @return remote port from the underlying WiFiClient.
    uint16_t remotePort() const
    {
        return remotePort_;
    }

    /// @return true if we successfully reconnected to the remote connection.
    bool reconnect()
    {
        if (connected())
        {
            return true;
        }

        // check for a previously disconnected connection and reject the
        // attempt to reconnect to it.
        if (remoteIP_ == INADDR_NONE || remotePort_ == 0)
        {
            return false;
        }
        return client_.connect(remoteIP_, remotePort_);
    }

    /// Disconnects the underlying WiFiClient connection. Note this will cause
    /// the @ref reconnect method to return false. After calling this method
    /// the instance of this adapter should be destroyed up rather than being
    /// reused.
    void stop()
    {
        LOG(VERBOSE, "Disconnecting from %s:%d on fd: %d",
            remoteIP_.toString().c_str(), remotePort_, client_.fd());
        client_.stop();
        remoteIP_ = INADDR_NONE;
        remotePort_ = 0;
    }

private:
    /// WiFiClient being wrapped.
    WiFiClient client_;
    IPAddress remoteIP_;
    uint16_t remotePort_;
};

#endif /* _FREERTOS_DRIVERS_ARDUINO_ESP32WIFI_HXX_ */
