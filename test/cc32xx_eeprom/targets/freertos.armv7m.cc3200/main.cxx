/** \copyright
 * Copyright (c) 2015, Stuart Baker
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
 * An application tests the Tiva/FreeRTOS implementation of the UART driver.
 *
 * @author Stuart Baker
 * @date 1 March 2015
 */

#include <unistd.h>
#include <fcntl.h>
#include "stdio.h"
#include <os/OS.hxx>

#include "nmranet_config.h"
#include "freertos_drivers/ti/CC32xxHelper.hxx"
#include "fs.h"

OVERRIDE_CONST(main_thread_stack_size, 2500);

void CheckResult(int res, int expected = 0) {
    if (res != expected) {
        printf("FAILED CHECK: expected %d actual %d\n", expected, res);
        usleep(1000000);
        DIE("expect");
    }
}


unsigned cache[256/4];

void runtest1() {
    printf("Current eeprom file contents:\n");
    for (int i = 0; i < 5; ++i) {
        string f = "/usr/eeprom.";
        f.push_back('0' + i);
        int32_t hnd = -133;
        int ret = sl_FsOpen((uint8_t*)f.c_str(), FS_MODE_OPEN_READ, NULL, &hnd);
        printf("hnd %ld ret %d\n", hnd, ret);
        CheckResult(ret);
        ret = sl_FsRead(hnd, 0, (uint8_t *)cache, 256);
        printf("file %d, read result %d\n", i, ret);
        for (unsigned j = 0; j < 256/4; ++j) {
            printf("0x%x ", cache[j]);
        }
        printf("\n");
    }
}

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    while (true) {
        printf("+Q\n");
        char req = getchar();
        printf("req: %c\n", req);
        switch(req) {
        case '1': {
            runtest1();
            break;
        }
            
        }
    }
    return 0;
}
