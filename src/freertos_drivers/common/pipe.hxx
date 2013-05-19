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
 * \file pipe.hxx
 * Interface and headers for the FreeRTOS-specific pipe functionality.
 *
 * @author Balazs Racz
 * @date 18 May 2013
 */

#ifndef _pipe_hxx_
#define _pipe_hxx_

#include <stddef.h>

#include <vector>
using std::vector;

//#include "devtab.h"
#include "os/os.h"
#include "os/OS.hxx"

struct devops;
typedef struct devops devops_t;
struct devtab;
typedef struct devtab devtab_t;

class PipeMember;

class Pipe
{
public:    
    Pipe(size_t unit);
    ~Pipe();

    //! Writes some data to all receivers of the pipe, except the one denoted
    //! by "skip_member".
    ssize_t WriteToAll(PipeMember* skip_member, const void* buf, size_t count);

    //! Adds a new member for the pipe. After this call all data will be
    //! transmitted ot the new member as well. Not thread-safe with writes.
    void RegisterMember(PipeMember* member);

    //! Removes a member. After this call no more new data will be transmitted
    //! to the member. Not thread-safe with writes.
    void UnregisterMember(PipeMember* member);

    size_t unit()
    {
	return unit_;
    }
private:
    //! The size (in bytes) of each read and write command. Only reads and
    //! writes in multiples of this unit are valid.
    size_t unit_;
    //! The individual receivers of data coming through the pipe. Members are externally owned.
    vector<PipeMember*> members_;
};

/**
   An interface class for channels where we forward data from a pipe.

   Various different pipe receivers will implement this interface.
 */
class PipeMember
{
public:
    virtual ~PipeMember()
    {
    }
    /**
       Writes bytes to the device.

       @param buf is the source buffer from whch to write bytes.

       @param count is the number of bytes to write. count is a multiple of the
    parent pipe's unit, otherwise implementations are allowed to drop data or
    die.

       Blocks until the write is complete (that is, all data is enqueued in a
    buffer which will drain as the output device's speed
    allows). Implementations may want to use a lock inside to avoid writes from
    multiple sources being interleaved.
    */
    virtual void write(const void* buf, size_t count) = 0;
};

//! Private data structure for pipe file nodes (aka virtual device nodes).
class VirtualPipeMember : public PipeMember
{
public:
    virtual ~VirtualPipeMember()
    {
    }

    VirtualPipeMember(Pipe* parent, int queue_length)
	: parent_(parent),
	  lock_(false),
	  read_queue_(NULL),
	  queue_length_(queue_length),
	  usage_count_(0)
    {
	// NOTE: at this point it is not certain that the parent object has
	// been constructed. Do not call anything there.
    }

    //! Handles data that comes from the parent Pipe.
    virtual void write(const void* buf, size_t count);

    //! Class containing static methods for pipe fd operations.
    class Ops;
    friend class Ops;
private:
    void Initialize();

    Pipe* parent_;
    OSMutex lock_;  //< Mutex for metadata in this class.
    OSMutex read_lock_;  //< Mutex for pipe_read() commands.
    OSMutex write_lock_;  //< Mutex for incoming write() (from the parent).
    os_mq_t read_queue_;  //< TX queue (from parent_ till fd read())
    int queue_length_;  //< length of TX queue (parent->unit() bytes each)
    int usage_count_;  //< Number of open file descriptors.
};

extern devops_t vdev_ops;
int vdev_init(devtab_t *dev);

/** Defines a pipe to forward data between real and virtual devices.

    example usage:
    DEFINE_PIPE(can_pipe, sizeof(struct can-frame));

    @param name must be a valic C identifier. This is the name under which to
    refer to this pipe.

    @param unit in bytes is the size of the base structure. All writes to the
    pipe must be a multiple of this unit.
 */
#define DEFINE_PIPE(name, unit) Pipe name(unit)

/** Adds a virtual device entry to a particular pipe.

    example usage:
    VIRTUAL_DEVTAB_ENTRY(vcan0, can_pipe, "/dev/vcan0", 16);

   @param name is a C identifier. You will not need this in the program.

    @param pipe is the identifier of the pipe to which to attach this device.

    @param path is a quoted string, the path of the virtual device

    @param qlen is the length of the receive buffer for this virtual device,
    measured in 'pipe unit' bytes.
*/
#define VIRTUAL_DEVTAB_ENTRY(name, pipe, path, qlen) \
    extern Pipe pipe; \
    VirtualPipeMember name(&pipe, qlen); \
    DEVTAB_ENTRY(name ## devtab, path, vdev_init, &vdev_ops, &name)

#endif //_pipe_hxx_

