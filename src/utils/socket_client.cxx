/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file socket_client.cxx
 *
 * Connects to a remote TCP socket.
 *
 * @author Balazs Racz
 * @date 28 Dec 2013
 */

/** @todo need an equivalent to gethostbyname_n on MacOS */
#if defined (__linux__) //|| defined (__MACH__)

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "utils/socket_listener.hxx"

#include "utils/macros.h"
#include "utils/logging.h"

int ConnectSocket(const char* host, int port) {
  struct hostent hnd, *hn;
  char buf[300];
  int local_errno;
  if (gethostbyname_r(host, &hnd, buf, sizeof(buf), &hn,
                      &local_errno) || !hn) {
    LOG_ERROR("gethostbyname failed for '%s': %s", host,
        strerror(local_errno));
    return -1;
  }

  struct sockaddr_in server;
  server.sin_family=hn->h_addrtype;
  server.sin_port=htons(port);
  memcpy((caddr_t)&(server.sin_addr),hn->h_addr, hn->h_length);

  int fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd < 0) {
    LOG_ERROR("socket: %s", strerror(errno));
    return -1;
  }

  int ret = connect(fd, (struct sockaddr *) &server, sizeof(server));
  if (ret < 0) {
    LOG_ERROR("connect: %s", strerror(errno));
    close(fd);
    return -1;
  }

  int val = 1;
  ERRNOCHECK("setsockopt(nodelay)",
             setsockopt(fd, IPPROTO_TCP, TCP_NODELAY,
                        &val, sizeof(val)));

  LOG(INFO, "Connected to %s:%d. fd=%d", host, port, fd);
  return fd;
}

#endif // __linux__
