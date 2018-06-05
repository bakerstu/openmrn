/** \copyright
 * Copyright (c) 2018, Balazs Racz
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
 * \file NumberDumper.hxx
 * Efficient implementation of dumping a lot of numbers to a log.
 *
 * @author Balazs Racz
 * @date 14 May 2018
 */

#ifndef _UTILS_NUMBERDUMPER_HXX_
#define _UTILS_NUMBERDUMPER_HXX_

#include "executor/StateFlow.hxx"
#include "utils/Singleton.hxx"
#include "utils/logging.h"


class NumberDumper : public Singleton<NumberDumper> {
public:
    NumberDumper(Service* s)
        : displayFlow_(s) {}

    void add(uint16_t data) {
        if (!nextChunk_) {
            nextChunk_ = displayFlow_.alloc();
            nextOfs_ = 0;
        }
        nextChunk_->data()->data[nextOfs_] = data;
        if (++nextOfs_ >= BUFSIZE) {
            displayFlow_.send(nextChunk_);
            nextChunk_ = nullptr;
        }
    }

    static constexpr uint16_t EOLN = 0xffffu;

private:
    static constexpr unsigned BUFSIZE = 128;

    struct Chunk {
        uint16_t data[BUFSIZE];
    };

    Buffer<Chunk>* nextChunk_ = nullptr;
    unsigned nextOfs_ = 0;

    class Displayer : public StateFlow<Buffer<Chunk>, QList<1> > {
    public:
        Displayer(Service* s) : StateFlow<Buffer<Chunk>, QList<1> >(s) {}

        Action entry() override {
            nextOfs_ = 0;
            outOfs_ = 0;
            //GLOBAL_LOG_OUTPUT((char*)message()->data()->data, sizeof(message()->data()->data));
            //return release_and_exit();
            return call_immediately(STATE(render));
        }

        Action render() {
            char* endp = output + outOfs_;
            char* limit = output + sizeof(output) - 12;
            char* lineLimit = output + lineOfs_ + MIN_LINE;
            while (true) {
                bool send_off = false;
                if (nextOfs_ >= BUFSIZE) {
                    send_off = true;
                } else if (message()->data()->data[nextOfs_] == EOLN) {
                    if (endp < lineLimit) {
                        *endp++ = '|';
                    } else if ((endp + MIN_LINE * 2) <= (output + sizeof(output)))
                    {
                        *endp++ = '\n';
                        lineLimit = endp + MIN_LINE;
                        lineOfs_ = endp - output;
                    } else {
                        send_off = true;
                        lineOfs_ = endp - output;
                    }
                } else if (endp > limit) {
                    send_off = true;
                }

                if (send_off) {
                    outOfs_ = endp - output;
                    lineOfs_ = ((int)outOfs_) - lineOfs_; // will be negative
                    GLOBAL_LOG_OUTPUT(output, outOfs_);
                    outOfs_ = 0;
                    endp = output;
                    lineLimit = output + lineOfs_ + MIN_LINE;
                }

                if (nextOfs_ >= BUFSIZE) {
                    return release_and_exit();
                }
                auto d = message()->data()->data[nextOfs_];
                if (d != EOLN) {
                    endp = unsigned_integer_to_buffer(d, endp);
                    *endp = ' ';
                    endp++;
                }
                outOfs_ = endp - output;
                if (++nextOfs_  % 30 == 0) return yield();
            }
        }

    private:
        uint16_t nextOfs_;
        uint16_t outOfs_;
        int16_t lineOfs_ = 0;
        char output[320];
        static constexpr uint8_t MIN_LINE = 60;
    };

    Displayer displayFlow_;
};




#endif // _UTILS_NUMBERDUMPER_HXX_
