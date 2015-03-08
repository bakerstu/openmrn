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
 * An application tests the Unix pipe() implementation on FreeRTOS.
 *
 * @author Stuart Baker
 * @date 31 January 2015
 */

#include <unistd.h>
#include <fcntl.h>
#include <os/OS.hxx>

/** Helper thread to send/receive from pipe */
class HelperThread : public OSThread
{
public:
    /** Constructor. */
    HelperThread(int fd, OSSem *sem)
        : OSThread()
        , fd(fd)
        , sem(sem)
    {}

    /** Destructor. */
    ~HelperThread() {}

private:
    /** Thread entry point
     * @return exit status
     */
    void *entry()
    {
        char data[11];
        int result = read(fd, data, 11);
        HASSERT(result == 11);
        HASSERT(strncmp(data, "test string", 11) == 0);
        close(fd);
        sem->post();

        return NULL;
    }

    /** read fd */
    int fd;

    /** unlock mutex on exit */
    OSSem *sem;

    DISALLOW_COPY_AND_ASSIGN(HelperThread);
};

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    OSSem sem(0);
    int fds[2];

    /* blocking mode test */
    for (int i = 0; i <10000; ++i)
    {
        pipe(fds);

        HelperThread *thread = new HelperThread(fds[0], &sem);

        thread->start("test", os_thread_getpriority(os_thread_self())+1, 1024);

        write(fds[1], "test string", 11);

        sem.wait();
        close(fds[1]);
        delete thread;
        usleep(2000);
    }

    /* set small pipe size, non-blocking */
    for (int i = 0; i < 10000; ++i)
    {
        char data;
        pipe(fds);
        fcntl(fds[0], F_SETPIPE_SZ, 1);
        fcntl(fds[0], F_SETFL, FNONBIO);
        fcntl(fds[1], F_SETFL, FNONBIO);
        int result = write(fds[1], "a", 1);
        HASSERT(result == 1);
        result = write(fds[1], "b", 1);
        HASSERT(result == 0);
        result = read(fds[0], &data, 1);
        HASSERT(result == 1);
        HASSERT(data == 'a');
        result = read(fds[0], &data, 1);
        HASSERT(result == 0);
        close(fds[0]);
        close(fds[1]);
    }
    
        //fcntl(fds[0], F_SETFL, FNONBIO);
    return 0;
}
