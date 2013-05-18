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

#include "devtab.h"
#include "os/os.h"


class PipeMember;

class Pipe
{
public:    
    Pipe(size_t unit);
    ~Pipe();

    //! Writes some data to all receivers of the pipe, except the one denoted
    //! by "skip_member".
    void WriteToAll(PipeMember* skip_member, const void* buf, size_t count);

    //! Adds a new member for the pipe. After this call all data will be
    //! transmitted ot the new member as well. Not thread-safe with writes.
    void RegisterMember(PipeMember* member);

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
    buffer which will drain as the output device's speed allows).
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

    VirtualPipeMember(Pipe* parent)
	: parent_(parent),
	  transmit_queue_(NULL)
    {
	// NOTE: at this point it is not certain that the parent object has
	// been constructed. Do not call anything there.
    }

    static int pipe_open(file_t* file, const char *path, int flags, int mode);
    static int pipe_close(file_t* file, node_t* node);
    static ssize_t pipe_read(file_t* file, void *buf, size_t count);
    static ssize_t pipe_write(file_t* file, const void *buf, size_t count);

private:
    void Initialize();

    Pipe* parent_;
    os_mq_t transmit_queue_;
    int queue_length_;
};

extern devops_t pipe_ops;



#endif //_pipe_hxx_

