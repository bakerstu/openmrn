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
 * \file pipe.cxx
 * Implementation of FreeRTOS-specific pipe funcitonality.
 *
 * @author Balazs Racz
 * @date 18 May 2013
 */

#include <stdint.h>
#include <unistd.h>

#include "pipe.hxx"

#include "os/os.h"

Pipe::Pipe(size_t unit)
    : unit_(unit)
{
}

Pipe::~Pipe()
{
}

void Pipe::RegisterMember(PipeMember* member)
{
    members_.push_back(member);
}

void Pipe::WriteToAll(PipeMember* skip_member, const void* buf, size_t count)
{
    configASSERT(count % unit_ == 0);
    for (PipeMember* member : members_)
    {
	if (member == skip_member) continue;
	member->write(buf, count);
    }
}


class PhysicalDevicePipeMember : public PipeMember
{
public:
    PhysicalDevicePipeMember(Pipe* parent, int fd, const char* rx_name, size_t stack_size)
	: fd_(fd)
    {
	os_thread_create(NULL, rx_name, 0, stack_size, &DeviceToPipeReaderThread, this);
    }

    virtual ~PhysicalDevicePipeMember()
    {
    }

    virtual void write(const void* buf, size_t count)
    {
	const uint8_t* bbuf = static_cast<const uint8_t*>(buf);
	ssize_t ret = 0;
	while (count > 0) {
	    ret = ::write(fd_, bbuf, count);
	    configASSERT(ret > 0);
	    count -= ret;
	    bbuf += ret;
	}
    }
    
private:
    static void* DeviceToPipeReaderThread(void* arg) {
	PhysicalDevicePipeMember* t = static_cast<PhysicalDevicePipeMember*>(arg);
	uint8_t buf[t->parent_->unit()];
	while(1)
	{
	    uint8_t* bbuf = buf;
	    int count = t->parent_->unit();
	    while (count > 0)
	    {
		ssize_t ret = ::read(t->fd_, bbuf, count);
		configASSERT(ret > 0);
		count -= ret;
		bbuf += ret;
	    }
	    t->parent_->WriteToAll(t, buf, t->parent_->unit());
	}
	return NULL;
    }
    
    //! File descriptor of the physical device.
    int fd_;

    //! Pipe to forward information to.
    Pipe* parent_;
};

static int ignore_ioctl(file_t *file, node_t *node, int key, void *data)
{
    return 0;
}


void VirtualPipeMember::Initialize()
{
    if (transmit_queue_) return;  // already initialized
    transmit_queue_ = os_mq_create(queue_length_, parent_->unit());
}

int VirtualPipeMember::pipe_open(file_t* file, const char *path, int flags, int mode)
{
    VirtualPipeMember* t = static_cast<VirtualPipeMember*>(file->dev->priv);
    t->Initialize();
    t->parent_->RegisterMember(t);
    return 0;
}

int VirtualPipeMember::pipe_close(file_t* file, node_t* node)
{
    return 0;
}
ssize_t VirtualPipeMember::pipe_read(file_t* file, void *buf, size_t count)
{
    return 0;
}
ssize_t VirtualPipeMember::pipe_write(file_t* file, const void *buf, size_t count)
{
    return 0;
}


DEVOPS(pipe_ops,
       VirtualPipeMember::pipe_open,
       VirtualPipeMember::pipe_close,
       VirtualPipeMember::pipe_read,
       VirtualPipeMember::pipe_write,
       ignore_ioctl);
