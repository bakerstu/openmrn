/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file LoggingBit.hxx
 *
 * Simple Bit event implementation that logs to the blinker or stderr.
 *
 * @author Balazs Racz
 * @date 13 Dec 2015
 */

#ifndef _BRACZ_CUSTOM_BLINKERFLOW_HXX_
#define _BRACZ_CUSTOM_BLINKERFLOW_HXX_

//static const uint64_t BLINKER_EVENT_ID = 0x0501010114FF2B08ULL;

class BlinkerFlow : public StateFlowBase {
 public:
  BlinkerFlow(nmranet::Node* node, uint64_t BLINKER_EVENT_ID)
      : StateFlowBase(node->iface()),
        state_(1),
        bit_(node, BLINKER_EVENT_ID, BLINKER_EVENT_ID + 1, &state_, (uint8_t)1),
        producer_(&bit_),
        sleepData_(this) {
    start_flow(STATE(blinker));
  }

 private:
  Action blinker() {
    state_ = !state_;
#ifdef __linux__
    LOG(INFO, "blink produce %d", state_);
#endif
    producer_.Update(&helper_, n_.reset(this));
    return wait_and_call(STATE(handle_sleep));
  }

  Action handle_sleep() {
    return sleep_and_call(&sleepData_, MSEC_TO_NSEC(1000), STATE(blinker));
  }

  uint8_t state_;
  nmranet::MemoryBit<uint8_t> bit_;
  nmranet::BitEventProducer producer_;
  nmranet::WriteHelper helper_;
  StateFlowTimer sleepData_;
  BarrierNotifiable n_;
};

#endif  // _BRACZ_CUSTOM_BLINKERFLOW_HXX_
