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
 * \file cpu_profile.cxx
 *
 * Helper functions for compiling CPU profiles from running targets.
 *
 * @author Balazs Racz
 * @date 6 June 2018
 */

#ifndef _FREERTOS_DRIVERS_COMMON_CPU_PROFILE_HXX_
#define _FREERTOS_DRIVERS_COMMON_CPU_PROFILE_HXX_

#include <unwind.h>

#define MAX_STRACE 20

#ifndef TRACE_BUFFER_LENGTH_WORDS
#define TRACE_BUFFER_LENGTH_WORDS 3000 // 12 kbytes
#endif

extern "C"
{
    extern void *stacktrace[MAX_STRACE];
    extern int strace_len;
    void call_unwind(void);
}

/// A custom unidirectional memory allocator so we can take traces from
/// interrupts.
class TraceAllocator
{
public:
    void *alloc(unsigned size)
    {
        size += 3;
        size /= 4;
        if (endOffset + size > TRACE_BUFFER_LENGTH_WORDS)
        {
            return nullptr;
        }
        void *ret = buffer + endOffset;
        endOffset += size;
        return ret;
    }

private:
    /// Storage of the trace buffer.
    unsigned buffer[TRACE_BUFFER_LENGTH_WORDS];
    /// index into buffer[] to denote first free element.
    unsigned endOffset{0};

public:
    /// how many times did we run into the MAX_STRACE limit.
    unsigned limitReached{0};
    /// How many times did we apply the backtrace hack to work around
    /// single-entry backtraces.
    unsigned singleLenHack{0};
} cpu_profile_allocator;

/// Linked list entry type for a call-stack backtrace.
struct trace
{
    /// For quick comparison of traces.
    unsigned hash : 24;
    /// Number of entries in the trace.
    unsigned len : 8;
    /// Link to the next trace entry.
    struct trace *next;
    /// total memory (bytes) allocated via this trace.
    unsigned total_size;
};

struct trace *all_traces = nullptr;

/// Computes a 24-bit hash code for a stack trace.
/// @param len is the number of entries in the stack trace (each 4 bytes)
/// @param buf is the first element (leaf) of the trace.
/// @return hash code.
unsigned hash_trace(unsigned len, unsigned *buf)
{
    unsigned ret = 0;
    for (unsigned i = 0; i < len; ++i)
    {
        ret *= (1 + 4 + 16);
        ret ^= buf[i];
    }
    return ret & 0xFFFFFF;
}

/// Finds if the current trace exists in the trace map.
/// @param hash is the previously computed hash value for the current trace.
/// @return nullptr if the current trace is not matching anything in the trace
/// buffer, otherwise the storage pointer to the copy of the current trace in
/// the trace buffer.
struct trace *find_current_trace(unsigned hash)
{
    for (struct trace *t = all_traces; t; t = t->next)
    {
        if (t->hash != (hash & 0xFFFFFF))
            continue;
        if (t->len != strace_len)
            continue;
        unsigned *payload = (unsigned *)(t + 1);
        if (memcmp(payload, stacktrace, strace_len * sizeof(stacktrace[0])) !=
            0)
            continue;
        return t;
    }
    return nullptr;
}

/// Adds the current trace to the trace buffer.
/// @param hash is the hash value ofthe current trace.
/// @return pointer to the trace in the trace buffer. The stats of the new
/// entry are not yet updated. Returns nullptr if the trace buffer is full.
struct trace *add_new_trace(unsigned hash)
{
    unsigned total_size =
        sizeof(struct trace) + strace_len * sizeof(stacktrace[0]);
    struct trace *t = (struct trace *)cpu_profile_allocator.alloc(total_size);
    if (!t)
        return nullptr;
    memcpy(t + 1, stacktrace, strace_len * sizeof(stacktrace[0]));
    t->hash = hash;
    t->len = strace_len;
    t->total_size = 0;
    t->next = all_traces;
    all_traces = t;
    return t;
}

/// Current stacktrace buffer. This will be written from an interrupt.
void *stacktrace[MAX_STRACE];
/// Number of entries in stacktrace.
int strace_len;

/// This struct definition mimics the internal structures of libgcc in
/// arm-none-eabi binary. It's not portable and might break in the future.
struct core_regs
{
    unsigned r[16];
};

/// This struct definition mimics the internal structures of libgcc in
/// arm-none-eabi binary. It's not portable and might break in the future.
typedef struct
{
    unsigned demand_save_flags;
    struct core_regs core;
} phase2_vrs;

/// We store what we know about the external context at interrupt entry in this
/// structure.
phase2_vrs main_context;
/// Saved value of the lr register at the exception entry.
unsigned saved_lr;

/// Takes registers from the core state and the saved exception context and
/// fills in the structure necessary for the LIBGCC unwinder.
void fill_phase2_vrs(volatile unsigned *fault_args)
{
    main_context.demand_save_flags = 0;
    main_context.core.r[0] = fault_args[0];
    main_context.core.r[1] = fault_args[1];
    main_context.core.r[2] = fault_args[2];
    main_context.core.r[3] = fault_args[3];
    main_context.core.r[12] = fault_args[4];
    // We add +2 here because first thing libgcc does with the lr value is
    // subtract two, presuming that lr points to after a branch
    // instruction. However, exception entry's saved PC can point to the first
    // instruction of a function and we don't want to have the backtrace end up
    // showing the previous function.
    main_context.core.r[14] = fault_args[6] + 2;
    main_context.core.r[15] = fault_args[6];
    saved_lr = fault_args[5];
    main_context.core.r[13] = (unsigned)(fault_args + 8); // stack pointer
}
extern "C"
{
    _Unwind_Reason_Code __gnu_Unwind_Backtrace(
        _Unwind_Trace_Fn trace, void *trace_argument, phase2_vrs *entry_vrs);
}

/// Static variable for trace_func.
void *last_ip;

/// Callback from the unwind backtrace function.
_Unwind_Reason_Code trace_func(struct _Unwind_Context *context, void *arg)
{
    void *ip;
    ip = (void *)_Unwind_GetIP(context);
    if (strace_len == 0)
    {
        // stacktrace[strace_len++] = ip;
        // By taking the beginning of the function for the immediate interrupt
        // we will attempt to coalesce more traces.
        // ip = (void *)_Unwind_GetRegionStart(context);
    }
    else if (last_ip == ip)
    {
        if (strace_len == 1 && saved_lr != _Unwind_GetGR(context, 14))
        {
            _Unwind_SetGR(context, 14, saved_lr);
            cpu_profile_allocator.singleLenHack++;
            return _URC_NO_REASON;
        }
        return _URC_END_OF_STACK;
    }
    if (strace_len >= MAX_STRACE - 1)
    {
        ++cpu_profile_allocator.limitReached;
        return _URC_END_OF_STACK;
    }
    // stacktrace[strace_len++] = ip;
    last_ip = ip;
    ip = (void *)_Unwind_GetRegionStart(context);
    stacktrace[strace_len++] = ip;
    return _URC_NO_REASON;
}

/// Called from the interrupt handler to take a CPU trace for the current
/// exception.
void take_cpu_trace()
{
    memset(stacktrace, 0, sizeof(stacktrace));
    strace_len = 0;
    last_ip = nullptr;
    phase2_vrs first_context = main_context;
    __gnu_Unwind_Backtrace(&trace_func, 0, &first_context);
    // This is a workaround for the case when the function in which we had the
    // exception trigger does not have a stack saved LR. In this case the
    // backtrace will fail after the first step. We manually append the second
    // step to have at least some idea of what's going on.
    if (strace_len == 0)
    {
        stacktrace[0] = (void*)main_context.core.r[15];
        strace_len++;
    }
    if (strace_len == 1)
    {
        main_context.core.r[14] = saved_lr;
        main_context.core.r[15] = saved_lr;
        last_ip = nullptr;
        __gnu_Unwind_Backtrace(&trace_func, 0, &main_context);
    }
    if (strace_len == 1)
    {
        stacktrace[1] = (void*)saved_lr;
        strace_len++;
    }
    unsigned h = hash_trace(strace_len, (unsigned *)stacktrace);
    struct trace *t = find_current_trace(h);
    if (!t)
    {
        t = add_new_trace(h);
    }
    if (t)
    {
        t->total_size += 1;
    }
}

/// Change this value to runtime disable and enable the CPU profile gathering
/// code.
bool enable_profiling = 0;
/// Current interrupt number for detecting per-interrupt costs.
volatile unsigned current_interrupt = 0;

/// Helper class for keeping track of current interrupt number.
class SetInterrupt
{
public:
    SetInterrupt(unsigned new_value)
    {
        old_value = current_interrupt;
        current_interrupt = new_value;
    }

    ~SetInterrupt()
    {
        current_interrupt = old_value;
    }

private:
    unsigned old_value;
};

/// Helper function to declare the CPU usage tick interrupt.
/// @param irq_handler_name is the name of the interrupt to declare, for example
/// timer4a_interrupt_handler.
/// @param CLEAR_IRQ_FLAG is a c++ statement or statements in { ... } that will
/// be executed before returning from the interrupt to clear the timer IRQ flag.
#define DEFINE_CPU_PROFILE_INTERRUPT_HANDLER(irq_handler_name, CLEAR_IRQ_FLAG) \
    extern "C"                                                                 \
    {                                                                          \
        void __attribute__((__noinline__)) load_monitor_interrupt_handler(     \
            volatile unsigned *exception_args, unsigned exception_return_code) \
        {                                                                      \
            if (enable_profiling)                                              \
            {                                                                  \
                fill_phase2_vrs(exception_args);                               \
                take_cpu_trace();                                              \
            }                                                                  \
            cpuload_tick(exception_return_code & 4                             \
                    ? 0                                                        \
                    : current_interrupt > 0 ? current_interrupt : 255);        \
            CLEAR_IRQ_FLAG;                                                    \
        }                                                                      \
        void __attribute__((__naked__)) irq_handler_name(void)                 \
        {                                                                      \
            __asm volatile("mov  r0, %0 \n"                                    \
                           "str  r4, [r0, 4*4] \n"                             \
                           "str  r5, [r0, 5*4] \n"                             \
                           "str  r6, [r0, 6*4] \n"                             \
                           "str  r7, [r0, 7*4] \n"                             \
                           "str  r8, [r0, 8*4] \n"                             \
                           "str  r9, [r0, 9*4] \n"                             \
                           "str  r10, [r0, 10*4] \n"                           \
                           "str  r11, [r0, 11*4] \n"                           \
                           "str  r12, [r0, 12*4] \n"                           \
                           "str  r13, [r0, 13*4] \n"                           \
                           "str  r14, [r0, 14*4] \n"                           \
                           :                                                   \
                           : "r"(main_context.core.r)                          \
                           : "r0");                                            \
            __asm volatile(" tst   lr, #4               \n"                    \
                           " ite   eq                   \n"                    \
                           " mrseq r0, msp              \n"                    \
                           " mrsne r0, psp              \n"                    \
                           " mov r1, lr \n"                                    \
                           " ldr r2,  =load_monitor_interrupt_handler  \n"     \
                           " bx  r2  \n"                                       \
                           :                                                   \
                           :                                                   \
                           : "r0", "r1", "r2");                                \
        }                                                                      \
    }

#endif // _FREERTOS_DRIVERS_COMMON_CPU_PROFILE_HXX_
