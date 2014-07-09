/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file async_datagram_test_helper.hxx
 *
 * Unit tests fixture classes for exercising datagram functionality.
 *
 * @author Balazs Racz
 * @date 23 Feb 2014
 */

#ifndef _UTILS_ASYNC_DATAGRAM_TEST_HELPER_HXX_
#define _UTILS_ASYNC_DATAGRAM_TEST_HELPER_HXX_

#include "utils/async_if_test_helper.hxx"
#include "nmranet/Datagram.hxx"
#include "nmranet/DatagramCan.hxx"

namespace nmranet {

class AsyncDatagramTest : public AsyncNodeTest
{
protected:
    AsyncDatagramTest() : datagram_support_(ifCan_.get(), 10, 2)
    {
    }

    CanDatagramService datagram_support_;
};

Pool* const g_incoming_datagram_allocator = mainBufferPool;

class TwoNodeDatagramTest : public AsyncDatagramTest
{
protected:
    enum
    {
        OTHER_NODE_ID = TEST_NODE_ID + 0x100,
        OTHER_NODE_ALIAS = 0x225,
    };

    void setup_other_node(bool separate_if)
    {
        if (separate_if)
        {
            otherIfCan_.reset(
                new IfCan(&g_executor, &can_hub0, 10, 10, 5));
            otherNodeIf_ = otherIfCan_.get();
            otherDatagramSupport_.reset(
                new CanDatagramService(otherNodeIf_, 10, 2));
            otherNodeDatagram_ = otherDatagramSupport_.get();
        }
        else
        {
            otherNodeIf_ = ifCan_.get();
            otherNodeDatagram_ = &datagram_support_;
        }
        otherNodeIf_->local_aliases()->add(OTHER_NODE_ID, OTHER_NODE_ALIAS);
        expect_packet(":X19100225N02010D000103;"); // node up
        otherNode_.reset(new DefaultNode(otherNodeIf_, OTHER_NODE_ID));
        wait();
    }

    void expect_other_node_lookup()
    {
        expect_packet(":X1070222AN02010D000103;"); // looking for DST node
        expect_packet(":X1949022AN02010D000103;"); // hard-looking for DST node
        expect_packet(":X19170225N02010D000103;"); // node ID verified
    }

    std::unique_ptr<DefaultNode> otherNode_;
    // Second objects if we want a bus-traffic test.
    std::unique_ptr<IfCan> otherIfCan_;
    IfCan* otherNodeIf_;
    std::unique_ptr<CanDatagramService> otherDatagramSupport_;
    CanDatagramService* otherNodeDatagram_;
};

} // namespace


#endif // _UTILS_ASYNC_DATAGRAM_TEST_HELPER_HXX_
