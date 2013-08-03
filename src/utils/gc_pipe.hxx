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
 * \file gc_pipe.hxx
 * Interface for creating gridconnect parser/renderer pipe components.
 *
 * @author Balazs Racz
 * @date 20 May 2013
 */

#ifndef _gc_pipe_hxx_
#define _gc_pipe_hxx_

class Pipe;

class GCAdapterBase
{
public:
    virtual ~GCAdapterBase()
    {
    }

    /**
       This function connects an ASCII (GridConnect-format) CAN adapter to a
       binary CAN adapter, performing the necessary format conversions
       inbetween.

       Specifically, it takes two Pipes as input, one carrying CAN frames in
       GridConnect protocol, and the other carrying CAN frames in the binary
       protocol.

       @param gc_side is the Pipe that has the ASCII GridConnect traffic.

       @param can_side is the Pipe that has the binary CAN traffic.

       @param double_bytes if true, any frame rendered into the GC protocol
       will have their characters doubled.

       @return a pointer to the created object. It can be deleted, which will
       terminate the link and unregister the link members from both pipes.
    */
    static GCAdapterBase* CreateGridConnectAdapter(Pipe* gc_side, Pipe* can_side, bool double_bytes);
};

#endif //_gc_pipe_hxx_
