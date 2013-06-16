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

/** A generalized pipe that allows connecting an arbitrary number of
   endpoints.

   Data sent by any endpoint will be received by all other endpoints. A special
   case of this (for two endpoints) is a regular pipe.
   
   Use-cases:

   1) A software interface can export a hardware device (with a device node
   path such as /dev/virtualUSB0). This is the use-case of a normal 2-way pipe,
   which allows protocol encapsulation without modifying lower-level code. For
   example it will be possible to create a stream over CAN packets for
   debugging output and export that stream as a device that can be opened as fd
   1 and 2 (stdout and stderr). This won't need any device drivers to be
   written.
   

   2) Creating multiple virtual CANbus adapters that send and receive traffic
   from the same physical bus, as if there were multiple physical adapters
   connected to the same wires.

   One great use of this is to attach alternate CANbus handlers to the device
   that normally OpenMRN library code reads.  There can be other traffic
   coexisting on the CANbus next to OpenLCB packets.  This traffic will have to
   be consumed by non-OpenMRN code, and will need its own reader thread.  It is
   not possible however to open the same device (e.g. /dev/can0) again, because
   that means that every packet will go to either the OpenMRN library code, or
   the application-specific code.  By creating a Pipe, adding the physical
   CANbus device to it and two virtual device nodes, it is possible to
   duplicate every incoming packet, route it to the appropriate handlers
   wihtout the handler code being aware of whether it is talking to a physical
   or virtual device. As a bonus, the handlers can talk to each other.

   Another use of this facility is for testing purposes, where multiple OpenMRN
   devices can be run, each with their own set of virtual nodes, on the same
   CANbus. They will be talking to each other as if it was different physical
   nodes (assuming te code is bug-free).

   A third use of this configuration is to create a "packet logger", where the
   OpenMRN node would capture all the trafic on the CANbus and log it to a host
   computer on a separate interface (e.g. USB port). This avoids the need for a
   separate CANbus sniffer.


   3) A CANbus 'hub'. 

   It is possible to add multiple physical devices to the same Pipe, which is a
   software-equivalent of connecting the wires together. All packets coming in
   on one bus will be copied to the other bus and vice versa. This allows to
   overcome the limitations of segment length or to bridge buses running at
   different speeds (at the expense of packet loss if the faster bus is
   saturated).

   It remains possible to add a virtual device to the same pipe and run OpenMRN
   stack on it.


   Example code:

   in hw_init.cxx

   DEFINE_PIPE(can_pipe0, sizeof(struct can_frame));
   VIRTUAL_DEVTAB_ENTRY(canp0v0, can_pipe0, "/dev/canp0v0", 16);
   VIRTUAL_DEVTAB_ENTRY(canp0v1, can_pipe0, "/dev/canp0v1", 16);

   in main.cxx
   
   DECLARE_PIPE(can_pipe0);

   int appl_main(int argc, char *argv[]) {
     can_pipe0.AddPhysicalDeviceToPipe("/dev/can0", "can0_rx_thread", 512);
     can_pipe0.AddPhysicalDeviceToPipe("/dev/can1", "can1_rx_thread", 512);
     int fd = open("/dev/canp0v0", O_RDWR);
     PacketLogger::init(fd);
     nmranet_if = nmranet_can_if_init(0x02010d000000ULL, "/dev/canp0v1", read, write);
     ...regular code...
   }
 */
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

    /** Adds a physical device to the members of this pipe.
        
        @param path is the path to the physical device, e.g. /dev/can0

        @param thread_name will be the name of the RX thread from the physical
        device

        @param stack_size will be the size of the RX thread stack.
     */
    void AddPhysicalDeviceToPipe(const char* path, const char* thread_name,
                                 int stack_size);

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

//! Use this if you need to refer to a pipe that was defined in a different compilation unit.
#define DECLARE_PIPE(name) extern Pipe name


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
