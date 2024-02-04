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
 * \file AsyncNotifiableBlock.hxx
 *
 * An advanced notifiable construct that acts as a fixed pool of
 * BarrierNotifiables. A stateflow can pend on acquiring one of them, use that
 * barrier, with it automatically returning to the next caller when the Barrier
 * goes out of counts.
 *
 * @author Balazs Racz
 * @date 18 Feb 2020
 */

#ifndef _EXECUTOR_ASYNCNOTIFIABLEBLOCK_HXX_
#define _EXECUTOR_ASYNCNOTIFIABLEBLOCK_HXX_

#include <memory>

#include "executor/Notifiable.hxx"
#include "utils/Queue.hxx"
#include "utils/logging.h"

#include "utils/Buffer.hxx"

/// A block of BarrierNotifiable objects, with an asynchronous allocation
/// call. Caller StateFlows can block on allocating a new entry, and then get
/// back a fresh BarrierNotifiable, which, upon being released will
/// automatically be reallocated to a waiting flow, if any.
class AsyncNotifiableBlock : private Notifiable, public QAsync
{
private:
    /// Notifiable class that can act as a BarrierNotifiable but also be
    /// enlisted in a queue.
    class QueuedBarrier : public BarrierNotifiable, public QMember
    {
    public:
        /// Notification implementation.
        ///
        /// Theory of operation: If this was the last notification (count goes
        /// from 1 to 0), we take the done_ pointer, cast it to the owning
        /// AsyncNotifiableBlock, and release outselves into the queue
        /// there. We keep the count at 1 at all times, which ensures that the
        /// done_ pointer remains pointing to the owner AsyncNotifiableBlock.
        void notify() override
        {
            AtomicHolder h(this);
            if (count_ == 1)
            {
                LOG(VERBOSE, "block notifiable %p returned pool size %u",
                    (BarrierNotifiable *)this,
                    (unsigned)mainBufferPool->total_size());
                auto *tgt = static_cast<AsyncNotifiableBlock *>(done_);
                tgt->insert(this);
            }
            else
            {
                --count_;
            }
        }

        /// Checks that there is exactly one count in here.
        void check_one_count()
        {
            HASSERT(count_ == 1);
        }
    };

public:
    /// Constructor. @param num_parallelism tells how many BarrierNotifiables
    /// we should have and hand out to callers requesting them.
    AsyncNotifiableBlock(unsigned num_parallelism)
        : count_(num_parallelism)
        , barriers_(new QueuedBarrier[num_parallelism])
    {
        for (unsigned i = 0; i < num_parallelism; ++i)
        {
            barriers_[i].reset(this);
            this->insert(&barriers_[i]);
        }
    }

    /// Destructor.
    ~AsyncNotifiableBlock();

    /// Turns an allocated entry from the QAsync into a usable
    /// BarrierNotifiable.
    /// @param entry a QMember that was allocated from *this.
    /// @return an initialized BarrierNotifiable with exactly one count, and
    /// done_ set up to be returned for further use.
    BarrierNotifiable *initialize(QMember *entry)
    {
        QueuedBarrier *b = static_cast<QueuedBarrier *>(entry);
        // We must be owning this entry.
        HASSERT(barriers_.get() <= b);
        HASSERT(b <= (barriers_.get() + count_));
        b->check_one_count();
        return b;
    }

    /// Notification implementation -- should never be called.
    void notify() override
    {
        DIE("Should not receive this notification");
    }

private:
    /// How many barriers do we have.
    unsigned count_;
    /// The pointer to the block of barriernotifiables.
    std::unique_ptr<QueuedBarrier[]> barriers_;

    DISALLOW_COPY_AND_ASSIGN(AsyncNotifiableBlock);
};

#endif // _EXECUTOR_ASYNCNOTIFIABLEBLOCK_HXX_
