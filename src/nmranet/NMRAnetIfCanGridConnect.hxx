/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file NMRAnetIfCanGcTcp.hxx
 * This file provides an NMRAnet interface specific to CAN over Grid Connect.
 *
 * @author Stuart W. Baker
 * @date 18 September 2013
 */

#ifndef _NMRAnetIfCanGcTCP_hxx_
#define _NMRAnetIfCanGcTCP_hxx_

#include "nmranet/NMRAnetIfCan.hxx"
#include "utils/GridConnect.hxx"

namespace NMRAnet
{

/** Provide a Grid Connect encoded extension to the normal CAN messaging.
 */
class IfCanGridConnect : public GridConnect, public IfCan
{
public:
    /** Constructor.
     * @param node_id node ID of interface
     * @param device description for this instance
     * @param read_double true if read encoding is two bytes per character, else
     * false
     * @param write_double true if read encoding is two bytes per character,
     * else false
     */
    static IfCanGridConnect* instance(NodeID node_id, const char* device,
                                      bool read_double = false,
                                      bool write_double = true)
    {
        if (read_double) {
            if (write_double) {
                return new IfCanGridConnect(node_id, device,
                                            GridConnect::read_double,
                                            GridConnect::write_double);
            } else {
                return new IfCanGridConnect(node_id, device,
                                            GridConnect::read_double,
                                            GridConnect::write);
            }
        } else {
            if (write_double) {
                return new IfCanGridConnect(node_id, device, GridConnect::read,
                                            GridConnect::write_double);
            } else {
                return new IfCanGridConnect(node_id, device, GridConnect::read,
                                            GridConnect::write);
            }
        }
    }

private:
    /** Constructor.
     * @param node_id node ID of interface
     * @param device description for this instance
     * @param read read method for this interface
     * @param write write method for this interface
     */
    IfCanGridConnect(NodeID node_id, const char* device,
                     ssize_t (*read)(int, void*, size_t),
                     ssize_t (*write)(int, const void*, size_t))
        : GridConnect(), IfCan(node_id, device, read, write)
    {
    }

    /** Default Constructor.
     */
    IfCanGridConnect();

    /** Default Destructor.
     */
    ~IfCanGridConnect()
    {
    }

    DISALLOW_COPY_AND_ASSIGN(IfCanGridConnect);
};
};

#endif /* _NMRAnetIfCanGcTCP_hxx_ */
