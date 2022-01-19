/** \copyright
 * Copyright (c) 2016, Stuart W Baker
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
 * \file DeviceFile.hxx
 *
 * Base class for implementing block devices and other file drivers which can
 * be seeked and addressed by absolute byte offsets.
 *
 * @author Stuart W. Baker
 * @date 16 July 2016
 */

#ifndef _FREERTOS_DRIVERS_COMMON_DEVICEFILE_HXX_
#define _FREERTOS_DRIVERS_COMMON_DEVICEFILE_HXX_

#include <cstdint>

#include "Devtab.hxx"

/**
 * Base class for implementing block devices and other file drivers which can
 * be seeked and addressed by absolute byte offsets.
 */
class DeviceFile : public Node
{
protected:
    /** Constructor.
     * @param name device file name
     */
    DeviceFile(const char *name)
        : Node(name)
    {
    }

    /** Destructor.
     */
    ~DeviceFile()
    {
    }

    /** Write to the DeviceFile.  NOTE!!! This is not necessarily atomic across
     * byte boundaries in the case of power loss.  The user should take this
     * into account as it relates to data integrity of a whole block.
     * @param index index within DeviceFile address space to start write
     * @param buf data to write
     * @param len length in bytes of data to write
     * @return number of bytes written upon success, -errno upon failure
     */
    virtual ssize_t write(unsigned int index, const void *buf, size_t len) = 0;

    /** Read from the DeviceFile.
     * @param index index within DeviceFile address space to start read
     * @param buf location to post read data
     * @param len length in bytes of data to read
     * @return number of bytes read upon success, -errno upon failure
     */
    virtual ssize_t read(unsigned int index, void *buf, size_t len) = 0;

    /** Open a device.
     * @param file new file reference to this device
     * @param path file or device name
     * @param flags open flags
     * @param mode open mode
     * @return 0 upon success, negative errno upon failure
     */
    int open(File* file, const char *path, int flags, int mode) OVERRIDE;

private:
    /** Read from a file or device.
     * @param file file reference for this device
     * @param buf location to place read data
     * @param count number of bytes to read
     * @return number of bytes read upon success, -errno upon failure
     */
    ssize_t read(File *file, void *buf, size_t count) OVERRIDE;

    /** Write to a file or device.
     * @param file file reference for this device
     * @param buf location to find write data
     * @param count number of bytes to write
     * @return number of bytes written upon success, -errno upon failure
     */
    ssize_t write(File *file, const void *buf, size_t count) OVERRIDE;

    void enable() OVERRIDE {} /**< function to enable device */
    void disable() OVERRIDE {} /**< function to disable device */

    /** Discards all pending buffers. Called after disable(). */
    void flush_buffers() OVERRIDE {}

    /** Default constructor.
     */
    DeviceFile();

    DISALLOW_COPY_AND_ASSIGN(DeviceFile);
};

#endif /* _FREERTOS_DRIVERS_COMMON_DEVICEFILE_HXX_ */

