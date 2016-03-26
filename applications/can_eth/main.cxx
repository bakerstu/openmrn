/** \copyright
 * Copyright (c) 2016, Sidney McHarg
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
 * \file main.cxx
 *
 * For now just a test bed for FreeRTOSTCP integration.  Eventually:
 *
 * A simple application to act as a CAN-TCP bridge. This application creates no
 * OpenLCB node, just forwards CAN packets to/from the host, in gridconnect
 * format.
 * Based on work by Balazs Racz
 *
 * @author Sidney McHarg
 * @date 26 March 2016
 */

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include "can_frame.h"
#include "executor/Executor.hxx"
#include "nmranet_config.h"
#include "os/os.h"
#include "utils/GridConnectHub.hxx"
#include "utils/Hub.hxx"
#include "utils/HubDevice.hxx"
#include "utils/blinker.h"

#include <netinet/tcp.h>
#include <sys/socket.h>
#include <arpa/inet.h>


Executor<1> g_executor("g_executor", 0, 1024);
Service g_service(&g_executor);
CanHubFlow can_hub0(&g_service);

OVERRIDE_CONST(gc_generate_newlines, 0);
OVERRIDE_CONST(can_tx_buffer_size, 8);
OVERRIDE_CONST(can_rx_buffer_size, 8);
OVERRIDE_CONST(serial_tx_buffer_size, 64);
OVERRIDE_CONST(serial_rx_buffer_size, 64);
#ifdef BOARD_LAUNCHPAD_EK
OVERRIDE_CONST(main_thread_stack_size, 2500);
#else
OVERRIDE_CONST(main_thread_stack_size, 900);
#endif

void *client_task(void *param)
{
	const char
		msg1[] = "Hello, you have connected to the test program\n",
		msg2[] = "\nNow closing connection\n";
	char buf[32];
	fd_set rd_set, temp_rd_set;
	struct timeval timeout = {5,0};
	int fd = (int) (param);
	int rslt = send(fd,msg1,sizeof(msg1),0);
	printf("send rslt=%d\n",rslt);
	FD_ZERO(&rd_set);
	FD_SET(fd,&rd_set);
	if (1)
	for (int i = 0; i < 5; i++)
	{
		temp_rd_set = rd_set;
		rslt = select(fd+1,&rd_set,nullptr,nullptr,&timeout);
		printf("select=%d\n",rslt);
		if (rslt <= 0)
			continue;
		if (FD_ISSET(fd,&rd_set))
		{
			// data to read
			rslt = recv(fd,buf,sizeof(buf),0);
			printf("recv rslt=%d\n",rslt);
			rslt = send(fd,buf,rslt,0);
			printf("send rslt=%d\n",rslt);
			(void) buf;
		}
	}
	FD_CLR(fd,&rd_set);
	send(fd,msg2,sizeof(msg2),0);
	sleep(1);
	close(fd);

	return NULL;
}

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char* argv[])
{
    int serial_fd = ::open("/dev/ser0", O_RDWR); // or /dev/ser0
    HASSERT(serial_fd >= 0);
    printf("Started\n");

    int socket_fd = socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
    printf("Socket created: %d (errno=%d)\n",socket_fd,errno);
    //sleep(5);
    HASSERT(socket_fd >= 0);

    struct sockaddr_in sin;
    memset(&sin,0,sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_port = htons(9000);
    int rslt = bind(socket_fd,(struct sockaddr *) &sin,sizeof(sin));
    printf("bind rslt=%d (errno=%d)\n",rslt,errno);

    rslt = listen(socket_fd,5);
    printf("listen rslt=%d errno=%d)\n",rslt,errno);


    create_gc_port_for_can_hub(&can_hub0, socket_fd);

    int can_fd = ::open("/dev/can0", O_RDWR);
    HASSERT(can_fd >= 0);

    FdHubPort<CanHubFlow> can_hub_port(
        &can_hub0, can_fd, EmptyNotifiable::DefaultInstance());

    while(1) {
    	socklen_t addr_len = sizeof(sin);
    	int client_fd = accept(socket_fd,(struct sockaddr *) &sin,&addr_len);
    	if (client_fd > 0)
    	{
    		printf("Connection accepted on %d\n",client_fd);
    		os_thread_create(NULL,"Client",configMAX_PRIORITIES-2,2048,client_task,(void *) client_fd);

    	}
        sleep(1);
        resetblink(1);
        sleep(1);
        resetblink(0);
    }
    return 0;
}
