/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file Receiver.hxx
 *
 * State flow for DCC packet reception and decoding.
 *
 * @author Balazs Racz
 * @date 30 November 2014
 */

#ifndef _DCC_RECEIVER_HXX_
#define _DCC_RECEIVER_HXX_

#include "executor/StateFlow.hxx"

namespace dcc {

class DccDecodeFlow : public StateFlowBase {
 public:
  DccDecodeFlow(Service* s, const char* dev)
      : StateFlowBase(s) {
    fd_ = ::open(dev, O_RDONLY | O_NONBLOCK);
    start_flow(STATE(register_and_sleep));
    timings_[DCC_ONE].set(52, 64);
    timings_[DCC_ZERO].set(95, 9900);
  }

 private:
  Action register_and_sleep() {
    ::ioctl(fd_, CAN_IOC_READ_ACTIVE, this);
    return wait_and_call(STATE(data_arrived));
  }

  Action data_arrived() {
    while (true) {
      uint32_t value;
      int ret = ::read(fd_, &value, sizeof(value));
      if (ret != 4) {
        return call_immediately(STATE(register_and_sleep));
      }
      MAP_GPIOPinWrite(LED_GREEN, 0xff);
      process_data(value);
    }
  }

  void process_data(uint32_t value) {
    switch (parseState_)
    {
      case UNKNOWN: {
        if (timings_[DCC_ONE].match(value)) {
          parseCount_ = 0;
          parseState_ = DCC_PREAMBLE;
          return;
        }
        break;
      }
      case DCC_PREAMBLE: {
        if (timings_[DCC_ONE].match(value)) {
          parseCount_++;
          return;
        }
        if (timings_[DCC_ZERO].match(value) && (parseCount_ >= 20)) {
          parseState_ = DCC_END_OF_PREAMBLE;
          return;
        }
        break;
      }
      case DCC_END_OF_PREAMBLE: {
        MAP_GPIOPinWrite(LED_YELLOW, 0xff);
        if (timings_[DCC_ZERO].match(value)) {
          parseState_ = DCC_DATA;
          parseCount_ = 1<<7;
          ofs_ = 0;
          data_[ofs_] = 0;
          return;
        }
        break;
      }
      case DCC_DATA: {
        if (timings_[DCC_ONE].match(value)) {
          parseState_ = DCC_DATA_ONE;
          return;
        }
        if (timings_[DCC_ZERO].match(value)) {
          parseState_ = DCC_DATA_ZERO;
          return;
        }
        break;
      }
      case DCC_DATA_ONE: {
        if (timings_[DCC_ONE].match(value)) {
          if (parseCount_) {
            data_[ofs_] |= parseCount_;
            parseCount_ >>= 1;
            parseState_ = DCC_DATA;
            return;
          } else {
            // end of packet 1 bit.
            dcc_packet_finished();
            parseState_ = DCC_MAYBE_CUTOUT;
            return;
          }
          return;
        }
        break;
      }
      case DCC_DATA_ZERO: {
        if (timings_[DCC_ZERO].match(value)) {
          if (parseCount_) {
            // zero bit into data_.
            parseCount_ >>= 1;
          } else {
            // end of byte zero bit. Packet is not finished yet.
            ofs_++;
            HASSERT(ofs_ < sizeof(data_));
            data_[ofs_] = 0;
            parseCount_ = 1<<7;
          }
          parseState_ = DCC_DATA;
          return;
        }
        break;
      }
      case DCC_MAYBE_CUTOUT: {
        if (value < timings_[DCC_ZERO].min_value) {
          HWREG(UART2_BASE + UART_O_CTL) |= UART_CTL_RXE;
        }
      }
    }
    parseState_ = UNKNOWN;
    return;
  }

  virtual void dcc_packet_finished() = 0;

  int fd_;
  uint32_t lastValue_ = 0;
  enum State {
    UNKNOWN,
    DCC_PREAMBLE,
    DCC_END_OF_PREAMBLE,
    DCC_DATA,
    DCC_DATA_ONE,
    DCC_DATA_ZERO,
    DCC_MAYBE_CUTOUT,
  };
  State parseState_ = UNKNOWN;
  uint32_t parseCount_ = 0;

protected:
  uint8_t data_[6];
  uint8_t ofs_;  // offset inside data_;

private:
  struct Timing {
    void set(int min_usec, int max_usec) {
      if (min_usec < 0) {
        min_value = 0;
      } else {
        min_value = usec_to_clock(min_usec);
      }
      if (max_usec < 0) {
        max_usec = UINT_MAX;
      } else {
        max_value = usec_to_clock(max_usec);
      }
    }

    bool match(uint32_t value_clocks) const {
      return min_value <= value_clocks && value_clocks <= max_value;
    }

    static uint32_t usec_to_clock(int usec) {
      return (configCPU_CLOCK_HZ / 1000000) * usec;
    }

    uint32_t min_value;
    uint32_t max_value;
  };

  enum TimingInfo {
    DCC_ONE = 0,
    DCC_ZERO,
    MAX_TIMINGS
  };
  Timing timings_[MAX_TIMINGS];
};

} // namespace dcc

#endif // _DCC_RECEIVER_HXX_
