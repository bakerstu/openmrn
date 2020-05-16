/** \copyright
 * Copyright (c) 2015, Stuart W Baker
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
 * \file Null.cxx
 * This file imlements a NULL device driver.
 *
 * @author Stuart W. Baker
 * @date 31 January 2015
 */

#include "Devtab.hxx"

#include <sys/stat.h>

/** default stdin */
const char *STDIN_DEVICE __attribute__ ((weak)) = "/dev/null";

/** default stdout */
const char *STDOUT_DEVICE __attribute__ ((weak)) = "/dev/null";

/** default stderr */
const char *STDERR_DEVICE __attribute__ ((weak)) = "/dev/null";

/** Null device instance */
class Null : public Device
{
public:
    /** Constructor */
    Null(const char* path) : Device(path)
    {
    }

    ~Null()
    {
    }

    /** Open method */
    int open(File *, const char *, int, int) OVERRIDE;
    /** Close method */
    int close(File *) OVERRIDE;
    /** Read method */
    ssize_t read(File *, void *, size_t) OVERRIDE;
    /** Write method */
    ssize_t write(File *, const void *, size_t) OVERRIDE;

    /** Get the status information of a file or device.
     * @param file file reference for this device
     * @param stat structure to fill status info into
     * @return 0 upon successor or negative error number upon error.
     */
    int fstat(File* file, struct stat *stat) OVERRIDE;

};

/// Filesystem node for the null device.
Null null("/dev/null");

/** Open a device.
 * @param file new file reference to this device
 * @param path file or device name
 * @param flags open flags
 * @param mode open mode
 * @return 0 upon success, negative errno upon failure
 */
int Null::open(File* file, const char *path, int flags, int mode)
{    
    return 0;
}

/** Close a device.
 * @param file file reference for this device
 * @return 0 upon success, negative errno upon failure
 */
int Null::close(File *file)
{    
    return 0;
}

/** Read from a file or device.
 * @param file file reference for this device
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, -1 upon failure with errno containing the cause
 */
ssize_t Null::read(File *file, void *buf, size_t count)
{
    // /dev/null returns EOF when trying to read from it.
    return 0;
}

/** Write to a file or device.
 * @param file file reference for this device
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -1 upon failure with errno containing the cause
 */
ssize_t Null::write(File *file, const void *buf, size_t count)
{    
    return count;
}

/** Get the status information of a file or device.
 * @param file file reference for this device
 * @param stat structure to fill status info into
 * @return 0 upon successor or negative error number upon error.
 */
int Null::fstat(File* file, struct stat *stat)
{
    memset(stat, 0, sizeof(*stat));
    return 0;
}

