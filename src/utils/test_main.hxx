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
#include <memory>
#include <string>
#include <functional>
#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "can_frame.h"
#include "executor/CallableFlow.hxx"
#include "executor/Executor.hxx"
#include "executor/Service.hxx"
#include "os/TempFile.hxx"
#include "os/os.h"
#include "utils/StringPrintf.hxx"

#ifdef WITHGPERFTOOLS
#include <gperftools/profiler.h>

std::function<void()> profiler_enable{&ProfilerEnable};
std::function<void()> profiler_disable{&ProfilerDisable};
#endif

int appl_main(int argc, char *argv[])
{
    testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}

bool mute_log_output = false;
extern "C" {

void log_output(char* buf, int size) {
    if (size <= 0 || mute_log_output) return;
    fwrite(buf, size, 1, stderr);
    fwrite("\n", 1, 1, stderr);
}

}


/// Global executor thread for tests.
extern Executor<1> g_executor;

#ifdef __EMSCRIPTEN__
Executor<1> g_executor{NO_THREAD()};

void os_emscripten_yield() {
    g_executor.loop_once();
}
#else
/// Global executor thread for tests. @todo maybe this should have 5 bands?
Executor<1> g_executor("ex_thread", 0, 1024);
#endif

#ifndef NO_GC_OPTIMIZE
/// Do not buffer gridconnect bytes when we are running in a test.
OVERRIDE_CONST(gridconnect_buffer_size, 1);
#endif

Service g_service(&g_executor);

/** Blocks the current thread until the main executor has run out of work.
 *
 * Use this function in unittest functions to wait for any asynchronous work
 * you may have scheduled on the main executor. This typically happens before
 * expectations. Also if you have stack-allocated objects that will schedule
 * themselves on the main executor, then you must call this function before
 * ending the scope that will deallocate them -- it is typical to have this as
 * the last command in a TEST_F. */
void wait_for_main_executor()
{
    std::unique_ptr<ExecutorGuard> guard(new ExecutorGuard(&g_executor));
    guard->wait_for_notification();
}

/** Delays the current thread until all asynchronous processing and all
 * pending timers have completed. */
void wait_for_main_timers()
{
    wait_for_main_executor();
    while (!g_executor.active_timers()->empty())
    {
        usleep(20000);
        wait_for_main_executor();
    }
    wait_for_main_executor();
}

/** Fixes race condition between test teardown and executor startup.
 *
 * Basically ensures that the main executor has started before trying to tear
 * it down. */
class ExecutorStartupFix {
public:
  ~ExecutorStartupFix() {
    wait_for_main_executor();
  }
} unused_executor_startup_guard_instance; ///< actual instance.

/** Utility class to help running a "pthread"-like thread in the main
 * executor. Helpful for emscripten compatibility. */
class ExecuteOnMainExecutor : public Executable {
public:
    /// Function type for a thread's main entry point.
    typedef void* thread_fn_t(void*);
    /** Schedules the function fn with argument arg on the main executor. Takes
     * ownership of *this. @param fn is the entry function of the
     * "thread". @param arg is the argument to pass to the function. */
    ExecuteOnMainExecutor(thread_fn_t *fn, void* arg)
        : fn_(fn), arg_(arg) {
        g_executor.add(this);
    }

    /// Runs the intended function with the given argument and then deletes
    /// this when done.
    void run() OVERRIDE {
        (*fn_)(arg_);
        delete this;
    }
private:
    thread_fn_t *fn_; ///< pointer to function to run.
    void* arg_; ///< argument to pass to function.
};

/// Helper class to run a lambda in the main executor.
class FnExecutable : public Executable
{
public:
    FnExecutable(std::function<void()> &&fn)
        : fn_(std::move(fn))
    {
    }

    void run() OVERRIDE
    {
        fn_();
        n.notify();
    }

    SyncNotifiable n;

private:
    std::function<void()> fn_;
};

/// Synchronously runs a function in the main executor.
void run_x(std::function<void()> fn)
{
    g_executor.sync_run(std::move(fn));
}

/// Runs some code in the constructor. Useful if custom code needs to be
/// injected into the constructor initialization order.
class RunInConstruct
{
public:
    RunInConstruct(std::function<void()> f)
    {
        f();
    }
};

/// Runs some code in the constructor on the main executor.
class RunInConstructOnMain
{
public:
    RunInConstructOnMain(std::function<void()> f)
    {
        run_x(f);
    }
};

/// Helper macro to make running certain test commands run on the main executor
/// simpler.
#define RX(statement) run_x([&](){ statement; })

/// Structure holding returned objects for an invoke_flow_nowait command.
template <class T> struct PendingInvocation
{
    /// Buffer sent to the flow.
    BufferPtr<T> b;
    /// Notifiable to wait for.
    SyncNotifiable notifiable;
    /// Barrier notifiable given to the buffer.
    BarrierNotifiable barrier {&notifiable};
    /// True if wait has been invoked.
    bool isWaited {false};

    ~PendingInvocation()
    {
        wait();
    }

    void wait()
    {
        if (isWaited)
        {
            return;
        }
        notifiable.wait_for_notification();
        isWaited = true;
    }
};

/// Executes a callable flow similar to invoke_flow(...) but does not wait for
/// the result to come back. Instead, returns a PendingInvocation object, where
/// there is a wait() method to be called.
template <class T, typename... Args>
std::unique_ptr<PendingInvocation<T>> invoke_flow_nowait(
    FlowInterface<Buffer<T>> *flow, Args &&...args)
{
    auto ret = std::make_unique<PendingInvocation<T>>();
    ret->b.reset(flow->alloc());
    ret->b->data()->reset(std::forward<Args>(args)...);
    ret->b->data()->done.reset(&ret->barrier);
    flow->send(ret->b->ref());
    return ret;
}

/** Utility class to block an executor for a while.
 *
 * Usage: add an instance of BlockExecutor to the executor you want to block,
 * (using @ref ExecutorBase::add) then call wait_for_blocked() and later
 * release_block() to unblock the executor.
 */
class BlockExecutor : public Executable
{
public:
    BlockExecutor()
    {
    }

    /** Creates a block against executor e and waits until the block
     * suceeds. @param e is the executor to block; ff e==null, then blocks
     * g_executor. */
    BlockExecutor(ExecutorBase *e)
    {
        if (e)
        {
            e->add(this);
        }
        else
        {
            g_executor.add(this);
        }
        wait_for_blocked();
    }

    virtual void run()
    {
        n_.notify();
#ifndef __EMSCRIPTEN__
        m_.wait_for_notification();
#endif
    }

    /** Blocks the current thread until the BlockExecutor manages to block the
    executor it was scheduled on. */
    void wait_for_blocked()
    {
        n_.wait_for_notification();
    }

    /** Releases the executor that was blocked. */
    void release_block()
    {
#ifndef __EMSCRIPTEN__
        m_.notify();
#endif
    }

private:
    /// notified (from the executor thread) when the block gets in place.
    SyncNotifiable n_;
    /// notified (from the test/operator thread) to release the block.
    SyncNotifiable m_;
};

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
class ScopedOverride
{
public:
    /// Constructor
    ///
    /// @param variable what to set temporarily
    /// @param new_value what should be the new value of variable during this
    /// code block.
    ///
    template <class T, typename U>
    ScopedOverride(T *variable, U new_value)
        : holder_(new Holder<T>(variable, new_value))
    {
    }

    /// Restores the original value.
    void restore()
    {
        holder_.reset();
    }

private:
    /// Virtual base class for the destructible holders.
    class HolderBase
    {
    public:
        virtual ~HolderBase()
        {
        }
    };

    /// Type-accurate class that holds the temporary variable with the old
    /// value, the pointer to the variable and restores the previous state upon
    /// destruction.
    template <class T> class Holder : public HolderBase
    {
    public:
        /// @param variable what to set temporarily
        /// @param new_value what should be the new value of variable during
        /// this code block.
        Holder(T *variable, T new_value)
            : variable_(variable)
            , oldValue_(*variable)
        {
            *variable = new_value;
        }

        ~Holder()
        {
            *variable_ = oldValue_;
        }

    private:
        /// Points to the variable that needs resetting.
        T *variable_;
        /// old value to reset variable_ to when destroyed.
        T oldValue_;
    };

    /// Smart ptr that will reset the variable to the previous value when going
    /// out of scope.
    std::unique_ptr<HolderBase> holder_;
};

/// Converts a character containing a hex digit to the value of that digit.
///
/// @param n character with lower or upper case hex digit.
///
/// @return int value of that digit.
int nibble_to_int(char n)
{
    if ('0' <= n && n <= '9')
    {
        return n - '0';
    }
    if ('a' <= n && n <= 'f')
    {
        return n - 'a' + 10;
    }
    if ('A' <= n && n <= 'F')
    {
        return n - 'A' + 10;
    }
    DIE("Unknown nibble arrived.");
}

/// Converts a hex string into the respective byte string.
/// @param hex a string containing hex digits. Separators between hex bytes are
/// allowed and ignored.
/// @return string containing bytes respective to the hex.
std::string hex2str(const char *hex)
{
    std::string ret;
    while (*hex && *(hex + 1))
    {
        if (*hex == ' ')
        {
            ++hex;
            continue;
        }
        ret.push_back((nibble_to_int(*hex) << 4) | (nibble_to_int(*(hex + 1))));
        hex += 2;
    }
    return ret;
}

const char HEXCHR[17] = "0123456789abcdef";

/// Converts a byte string into the hexadecimal representation, with no
/// separators.
/// @param s Arbitrary byte payload.
/// @return string containing two hex digits per incoming byte, with the
/// respective value..
std::string str2hex(const string &s)
{
    std::string ret;
    for (char c : s)
    {
        ret.push_back(HEXCHR[c >> 4]);
        ret.push_back(HEXCHR[c & 0xf]);
    }
    return ret;
}

#endif // _UTILS_TEST_MAIN_HXX_
