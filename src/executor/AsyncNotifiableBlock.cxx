/** \copyright
 * Copyright (c) 2020, Balazs Racz
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
 * \file AsyncNotifiableBlock.cxx
 *
 * An advanced notifiable construct that acts as a fixed pool of
 * BarrierNotifiables. A stateflow can pend on acquiring one of them, use that
 * barrier, with it automatically returning to the next caller when the Barrier
 * goes out of counts.
 *
 * @author Balazs Racz
 * @date 18 Feb 2020
 */

#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif

#include "AsyncNotifiableBlock.hxx"

#include "os/sleep.h"

AsyncNotifiableBlock::~AsyncNotifiableBlock()
{
    // Recollects all notifiable instances, including waiting as long as needed
    // if there are some that have not finished yet.
    for (unsigned i = 0; i < count_; ++i)
    {
        while (true)
        {
            QMember *m = next().item;
            if (!m)
            {
                LOG(VERBOSE,
                    "shutdown async notifiable block: waiting for returns");
                microsleep(100);
            }
            else
            {
                HASSERT(initialize(m)->abort_if_almost_done());
                break;
            }
        }
    }
}
