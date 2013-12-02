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

#include "utils/logging.h"
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>

#include <algorithm>
using std::remove;

#include "pipe.hxx"

#include "os/os.h"
#include "os/OS.hxx"

#ifndef configASSERT
#include <assert.h>
#define configASSERT assert
#endif

Pipe::Pipe(size_t unit) : unit_(unit) {}

Pipe::~Pipe() {}

void Pipe::RegisterMember(PipeMember* member) { members_.push_back(member); }

void Pipe::UnregisterMember(PipeMember* member) {
  members_.erase(remove(members_.begin(), members_.end(), member),
                 members_.end());
}

ssize_t Pipe::WriteToAll(PipeMember* skip_member, const void* buf,
                         size_t count) {
  configASSERT(count % unit_ == 0);
  for (PipeMember* member : members_) {
    if (member == skip_member) continue;
    member->write(buf, count);
  }
  return count;
}

class PhysicalDevicePipeMember : public PipeMember {
 public:
  PhysicalDevicePipeMember(Pipe* parent, int fd_read, int fd_write,
                           const char* rx_name, size_t stack_size)
      : fd_read_(fd_read), fd_write_(fd_write), parent_(parent) {
    os_thread_create(&read_thread_, rx_name, 0, stack_size,
                     &DeviceToPipeReaderThread, this);
  }

  virtual ~PhysicalDevicePipeMember() {
    // There is no current possibility to stop the thread we created. Let's
    // crash.
    abort();
  }

  virtual void write(const void* buf, size_t count) {
    const uint8_t* bbuf = static_cast<const uint8_t*>(buf);
    ssize_t ret = 0;
    while (count > 0) {
      ret = ::write(fd_write_, bbuf, count);
      if (!ret) {
        LOG(ERROR, "EOF writing fd %d.", fd_write_);
      }
      configASSERT(ret > 0);
      count -= ret;
      bbuf += ret;
    }
  }

 private:
  static void* DeviceToPipeReaderThread(void* arg) {
    PhysicalDevicePipeMember* t = static_cast<PhysicalDevicePipeMember*>(arg);
    uint8_t buf[t->parent_->unit()];
    while (1) {
      uint8_t* bbuf = buf;
      int count = t->parent_->unit();
      while (count > 0) {
        ssize_t ret = ::read(t->fd_read_, bbuf, count);
        if (!ret) {
          LOG(ERROR, "EOF reading pipe fd %d.\n", t->fd_read_);
          t->parent_->UnregisterMember(t);
          ::close(t->fd_read_);
          if (t->fd_write_ != t->fd_read_) {
            ::close(t->fd_write_);
          }
          return NULL;
        }
        configASSERT(ret > 0);
        count -= ret;
        bbuf += ret;
      }
      t->parent_->WriteToAll(t, buf, t->parent_->unit());
    }
    return NULL;
  }

  //! File descriptor from the physical device.
  int fd_read_;
  //! File descriptor to the physical device.
  int fd_write_;

  //! Pipe to forward information to.
  Pipe* parent_;

  //! Thread handle for reader thread.
  os_thread_t read_thread_;

  //! Protects writes to the device.
  OSMutex lock_;
};

void Pipe::AddPhysicalDeviceToPipe(const char* path, const char* thread_name,
                                   int stack_size) {
  int fd = ::open(path, O_RDWR);
  configASSERT(fd >= 0);
  AddPhysicalDeviceToPipe(fd, fd, thread_name, stack_size);
}

void Pipe::AddPhysicalDeviceToPipe(int fd_read, int fd_write,
                                   const char* thread_name, int stack_size) {
  // The new member is effectively leaked here. The problem is that we cannot
  // destruct physical pipe members because they have a thread.
  RegisterMember(new PhysicalDevicePipeMember(this, fd_read, fd_write,
                                              thread_name, stack_size));
}

#ifdef __linux__
void Pipe::AddVirtualDeviceToPipe(const char* thread_name, int stack_size,
                                  int fd[2]) {
  int fromfd[2];
  int tofd[2];
  assert(!pipe(fromfd));
  assert(!pipe(tofd));

  AddPhysicalDeviceToPipe(fromfd[0], tofd[1], thread_name, stack_size);
  fd[0] = tofd[0];
  fd[1] = fromfd[1];
}
#endif

#ifdef __FreeRTOS__

#include "devtab.h"

void VirtualPipeMember::Initialize() {
  OSMutexLock l(&lock_);
  if (read_queue_) return;  // already initialized
  read_queue_ = os_mq_create(queue_length_, parent_->unit());
}

class VirtualPipeMember::Ops {
 public:
  static int pipe_open(file_t* file, const char* path, int flags, int mode);
  static int pipe_close(file_t* file, node_t* node);
  static ssize_t pipe_read(file_t* file, void* buf, size_t count);
  static ssize_t pipe_write(file_t* file, const void* buf, size_t count);
};

int VirtualPipeMember::Ops::pipe_open(file_t* file, const char* path, int flags,
                                      int mode) {
  if (flags & O_NONBLOCK) {
    // Pipes do not currently support nonblocking mode. This restriction
    // comes from the interface of PipeMember -- it does not support
    // nonblocking writes. A possible option to implement it would be to
    // start a separate RX thread in case of nonblocking IO requested, add
    // an RX queue and pass on the responsibility of the blocking
    // Pipe::WriteToAll call to the specialized thread.
    return -EINVAL;
  }
  VirtualPipeMember* t = static_cast<VirtualPipeMember*>(file->dev->priv);
  t->Initialize();  // Will lock inside.
  OSMutexLock l(&t->lock_);
  if (t->usage_count_++ == 0) {
    t->parent_->RegisterMember(t);
  }
  return 0;
}

int VirtualPipeMember::Ops::pipe_close(file_t* file, node_t* node) {
  VirtualPipeMember* t = static_cast<VirtualPipeMember*>(file->dev->priv);
  OSMutexLock l(&t->lock_);
  configASSERT(t->usage_count_ > 0);
  if (--t->usage_count_ <= 0) {
    t->parent_->UnregisterMember(t);
  }
  return 0;
}

ssize_t VirtualPipeMember::Ops::pipe_read(file_t* file, void* buf,
                                          size_t count) {
  VirtualPipeMember* t = static_cast<VirtualPipeMember*>(file->dev->priv);
  OSMutexLock l(&t->read_lock_);
  uint8_t* bbuf = static_cast<uint8_t*>(buf);
  ssize_t result = 0;
  while (count >= t->parent_->unit()) {
    os_mq_receive(t->read_queue_, bbuf);
    count -= t->parent_->unit();
    bbuf += t->parent_->unit();
    result += t->parent_->unit();
  }
  return result;
}

ssize_t VirtualPipeMember::Ops::pipe_write(file_t* file, const void* buf,
                                           size_t count) {
  VirtualPipeMember* t = static_cast<VirtualPipeMember*>(file->dev->priv);
  return t->parent_->WriteToAll(t, buf, count);
}

void VirtualPipeMember::write(const void* buf, size_t count) {
  OSMutexLock l(&write_lock_);
  const uint8_t* bbuf = static_cast<const uint8_t*>(buf);
  while (count >= parent_->unit()) {
    os_mq_send(read_queue_, bbuf);
    count -= parent_->unit();
    bbuf += parent_->unit();
  }
}

static int ignore_ioctl(file_t* file, node_t* node, int key, void* data) {
  return 0;
}

int vdev_init(devtab_t* dev) {
  // Nothing to init here, because the constructor takes care of
  // initialization.
  return 0;
}

DEVOPS(vdev_ops, VirtualPipeMember::Ops::pipe_open,
       VirtualPipeMember::Ops::pipe_close, VirtualPipeMember::Ops::pipe_read,
       VirtualPipeMember::Ops::pipe_write, ignore_ioctl);
#endif
