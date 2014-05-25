/** \copyright
 * Copyright (c) 2013, Stuart W Baker
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file NMRAnetMemoryConfig.hxx
 * This file defines NMRAnet Memory Configuration Protocol(s).
 *
 * @author Stuart W. Baker
 * @date 13 October 2013
 */

#ifndef _NMRAnetMemoryhxx_
#define _NMRAnetMemoryhxx_

#include "utils/BufferQueue.hxx"
#include "nmranet/NMRAnetIf.hxx"

namespace nmranet
{

class Node;

class MemoryConfig
{
public:
    /** Field offsets within datagram
     */
    enum offsets
    {
        COMMAND         = 0,
        ADDRESS         = 1,
        AVAILABLE       = 1,
        ADDRESS_SPACE   = 1,
        ADDRESS_HIGHEST = 2,
        LENGTHS         = 3,
        HIGHEST         = 4,
        LOWEST          = 5,
        SPACE           = 5,
        COUNT           = 5,
        DATA            = 5,
        FLAGS           = 6,
        ADDRESS_LOWEST  = 7,
        DESCRIPTION     = 11,

        COUNT_MASK      = 0x7F /**< the upper bit of the count is reserved */
    };


protected:
    /** Constructor.
     * @param cdi Configuration Data, else NULL to use default CDI
     */
    MemoryConfig(uint8_t *cdi);
    
    /** Default Destructor
     */
    ~MemoryConfig()
    {
    }

    /** Process a memory configuration datagram.
     * @param datagram datagram to process, this pointer is stale upon return
     */
    void process(BufferBase *buffer);

private:
    /** Helper function for replyoing to a memory config request.
     * @param from node we are replying to
     * @param command configuration command
     * @param address address within address space the data coresponds to
     * @param data location in memory to copy address space data from
     * @param count number of bytes to send
     */
    void reply(NodeHandle from,
               const void *command, const void *address,
               const void *data, const uint8_t count);
    
    /** Default hander for memory config all.
     * @param address offset in bytes to grab
     * @param count number of bytes to gram [in], number of bytes available [out]
     * @return pointer in memory to address space + offset
     */
    static void *all_memory(uint32_t address, uint8_t *count);

    /** Global CDI that is used if not overridden at the Node level */
    static const uint8_t globalCdi[];

    /** Global CDI size that is used if not overridden at the Node level */
    static size_t globalCdiSize;

    /** Node specific CDI */
    const uint8_t *cdi;
    
    /** size of Node specific CDI string */
    size_t cdiSize;
    
    /** Default Constructor.
     */
    MemoryConfig();
    
    DISALLOW_COPY_AND_ASSIGN(MemoryConfig);
};

};

#endif /* _NMRAnetMemoryhxx_ */

