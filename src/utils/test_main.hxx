/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file test_main.hxx
 *
 * Include this file into your unittest to define the necessary symbols and
 * main function.
 *
 * @author Balazs Racz
 * @date 3 Nov 2013
 */

#ifdef _UTILS_TEST_MAIN_HXX_
#error Only ever include test_main into the main unittest file.
#else
#define _UTILS_TEST_MAIN_HXX_

#include "nmranet_config.h"

#include <stdio.h>
#include <stdarg.h>
#include <string>
#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "os/os.h"
#include "utils/pipe.hxx"
#include "nmranet_can.h"

namespace testing {
/** Conveninence utility to do a printf directly into a C++ string. */
string StringPrintf(const char* format, ...) {
  static const int kBufSize = 1000;
  char buffer[kBufSize];
  va_list ap;

  va_start(ap, format);
  int n = vsnprintf(buffer, kBufSize, format, ap);
  va_end(ap);
  HASSERT(n >= 0);
  if (n < kBufSize) {
    return string(buffer, n);
  }
  string ret(n + 1, 0);
  va_start(ap, format);
  n = vsnprintf(&ret[0], ret.size(), format, ap);
  va_end(ap);
  HASSERT(n >= 0);
  ret.resize(n);
  return ret;
}
}

using testing::StringPrintf;

int appl_main(int argc, char* argv[]) {
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}

ThreadExecutor g_executor("g_executor", 0, 2000);
DEFINE_PIPE(can_pipe0, &g_executor, sizeof(struct can_frame));

void WaitForMainExecutor() {
  while (!g_executor.empty()) {
    usleep(100);
  }
}

/** Overrides the value of a variable and restores it to the original value
 * when destructed. Useful for changing flags for a single test only.
 *
 * Usage:
 * {
 *    ScopedOverride ov(&DATAGRAM_RESPONSE_TIMEOUT_NSEC, 100000);
 *    ... test code assuming new value ...
 * }
 * ... now the original value is restored.
 */
class ScopedOverride {
 public:
  template <class T> ScopedOverride(T* variable, T new_value)
      : holder_(new Holder<T>(variable, new_value)) {}

 private:
  class HolderBase { public: virtual ~HolderBase() {} };

  template<class T> class Holder : public HolderBase {
   public:
    Holder(T* variable, T new_value)
        : variable_(variable), oldValue_(*variable) {
      *variable = new_value;
    }

    ~Holder() {
      *variable_ = oldValue_;
    }
   private:
    T* variable_;
    T oldValue_;
  };

  std::unique_ptr<HolderBase> holder_;
};

extern "C" {

const char *nmranet_manufacturer = "Stuart W. Baker";
const char *nmranet_hardware_rev = "N/A";
const char *nmranet_software_rev = "0.1";

const size_t main_stack_size = 2560;
const size_t ALIAS_POOL_SIZE = 2;
const size_t DOWNSTREAM_ALIAS_CACHE_SIZE = 2;
const size_t UPSTREAM_ALIAS_CACHE_SIZE = 2;
const size_t DATAGRAM_POOL_SIZE = 10;
const size_t CAN_RX_BUFFER_SIZE = 1;
const size_t CAN_TX_BUFFER_SIZE = 32;
const size_t SERIAL_RX_BUFFER_SIZE = 16;
const size_t SERIAL_TX_BUFFER_SIZE = 16;
const size_t DATAGRAM_THREAD_STACK_SIZE = 512;
const size_t CAN_IF_READ_THREAD_STACK_SIZE = 1024;
const size_t COMPAT_EVENT_THREAD_STACK_SIZE = 1024;
const size_t WRITE_FLOW_THREAD_STACK_SIZE = 1024;

}

#endif // _UTILS_TEST_MAIN_HXX_
