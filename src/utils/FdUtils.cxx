/** \copyright
 * Copyright (c) 2023, Balazs Racz
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
 * \file FdUtils.cxx
 *
 * Helper functions for dealing with posix fds.
 *
 * @author Balazs Racz
 * @date 29 Dec 2023
 */

#include "FdUtils.hxx"

#ifdef __linux__
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/stat.h>
#include <termios.h> /* tc* functions */
#endif

#include "nmranet_config.h"

/// Performs a system call on an fd. If an error is returned, prints the error
/// using the log mechanism, but otherwise ignores it.
/// @param where user-readable text printed with the error, e.g. "setsockopt"
/// @param callfn system call, like ::setsockopt
/// @param fd file descriptor (int)
/// @param args... all other arguments to callfn
#define PCALL_LOGERR(where, callfn, fd, args...)                               \
    do                                                                         \
    {                                                                          \
        int ret = callfn(fd, args);                                            \
        if (ret < 0)                                                           \
        {                                                                      \
            char buf[256];                                                     \
            strerror_r(errno, buf, sizeof(buf));                               \
            LOG_ERROR("fd %d %s: %s", fd, where, buf);                         \
        }                                                                      \
    } while (0)

/// Optimizes the kernel settings like socket and TCP options for an fd
/// that is an outgoing TCP socket.
/// @param fd socket file descriptor.
void FdUtils::optimize_socket_fd(int fd)
{
#ifdef __linux__
    const int rcvbuf = config_gridconnect_tcp_rcv_buffer_size();
    if (rcvbuf > 1)
    {
        PCALL_LOGERR("setsockopt SO_RCVBUF", ::setsockopt, fd, SOL_SOCKET,
            SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    }
    const int sndbuf = config_gridconnect_tcp_snd_buffer_size();
    if (sndbuf > 1)
    {
        PCALL_LOGERR("setsockopt SO_SNDBUF", ::setsockopt, fd, SOL_SOCKET,
            SO_SNDBUF, &sndbuf, sizeof(sndbuf));
        int ret = 0;
        socklen_t retsize = sizeof(ret);
        ::getsockopt(fd, SOL_SOCKET, SO_SNDBUF, &ret, &retsize);
        LOG(ALWAYS, "fd %d sndbuf %d", fd, ret);
    }
    const int lowat = config_gridconnect_tcp_notsent_lowat_buffer_size();
    if (lowat > 1)
    {
        PCALL_LOGERR("setsockopt tcp_notsent_lowat", ::setsockopt, fd,
            IPPROTO_TCP, TCP_NOTSENT_LOWAT, &lowat, sizeof(lowat));
        int ret = 0;
        socklen_t retsize = sizeof(ret);
        ::getsockopt(fd, IPPROTO_TCP, TCP_NOTSENT_LOWAT, &ret, &retsize);
        LOG(ALWAYS, "fd %d lowat %d", fd, ret);
    }
#endif
}

/// Sets the kernel settings like queuing and terminal settings for an fd
/// that is an outgoing tty.
/// @param fd tty file descriptor.
void FdUtils::optimize_tty_fd(int fd)
{
#ifdef __linux__
    // Sets up the terminal in raw mode. Otherwise linux might echo
    // characters coming in from the device and that will make
    // packets go back to where they came from.
    HASSERT(!tcflush(fd, TCIOFLUSH));
    struct termios settings;
    HASSERT(!tcgetattr(fd, &settings));
    cfmakeraw(&settings);
    cfsetspeed(&settings, B115200);
    HASSERT(!tcsetattr(fd, TCSANOW, &settings));
#endif
}

/// For an fd that is an outgoing link, detects what kind of file
/// descriptor this is and calls the appropriate optimize call for it.
void FdUtils::optimize_fd(int fd)
{
#ifdef __linux__
    struct stat statbuf;
    fstat(fd, &statbuf);

    if (S_ISSOCK(statbuf.st_mode))
    {
        optimize_socket_fd(fd);
    }
    else if (isatty(fd))
    {
        optimize_tty_fd(fd);
    }
#endif
}
