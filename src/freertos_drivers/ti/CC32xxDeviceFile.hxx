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
 * \file CC32xxDeviceFile.hxx
 * This implements a CC32xx specific device file abstraction.
 *
 * @author Stuart W. Baker
 * @date 16 July 2016
 */

#ifndef _FREERTOS_DRIVERS_TI_CC32XXDEVICEFILE_HXX_
#define _FREERTOS_DRIVERS_TI_CC32XXDEVICEFILE_HXX_

#include "DeviceFile.hxx"

/** Common base class for all CC32xxDeviceFile access.
 */
class CC32xxDeviceFile : public DeviceFile
{
public:
    /** Constructor.
     * @param name device file name
     * @param max_size_on_create this is the maximum size of the file allocated
     *                           when it is created
     */
    CC32xxDeviceFile(const char *name, size_t max_size_on_create = 3000)
        : DeviceFile(name)
        , handle(-1)
        , size(0)
        , maxSize(0)
        , maxSizeOnCreate(max_size_on_create)
        , writeEnable(false)
    {
    }

    /** Destructor.
     */
    ~CC32xxDeviceFile()
    {
    }

protected:
    /** Write to the CC32xxDeviceFile.  NOTE!!! This is not necessarily atomic
     * across
     * byte boundaries in the case of power loss.  The user should take this
     * into account as it relates to data integrity of a whole block.
     * @param index index within CC32xxDeviceFile address space to start write
     * @param buf data to write
     * @param len length in bytes of data to write
     * @return number of bytes written upon success, -errno upon failure
     */
    ssize_t write(unsigned int index, const void *buf, size_t len) override;

    /** Read from the CC32xxDeviceFile.
     * @param index index within CC32xxDeviceFile address space to start read
     * @param buf location to post read data
     * @param len length in bytes of data to read
     * @return number of bytes read upon success, -errno upon failure
     */
    ssize_t read(unsigned int index, void *buf, size_t len) override;

private:
    /** Open a device.
     * @param file new file reference to this device
     * @param path file or device name
     * @param flags open flags
     * @param mode open mode
     * @return 0 upon success, negative errno upon failure
     */
    int open(File* file, const char *path, int flags, int mode) override;


    /** Get the status information of a file or device.
     * @param file file reference for this device
     * @param stat structure to fill status info into
     * @return 0 upon successor or negative error number upon error.
     */
    int fstat(File* file, struct stat *stat) override;
    
    void disable() OVERRIDE; /**< function to disable device */

    /** file handle */
    int32_t handle;

    /** size of file in bytes */
    uint32_t size;

    /** max size of file in bytes */
    uint32_t maxSize;

    /** max size of file upon creation */
    int32_t maxSizeOnCreate;

    /** is the file open for write */
    bool writeEnable;

    /** Default constructor.
     */
    CC32xxDeviceFile();

    DISALLOW_COPY_AND_ASSIGN(CC32xxDeviceFile);
};

#endif /* _FREERTOS_DRIVERS_TI_CC32XXDEVICEFILE_HXX_ */

