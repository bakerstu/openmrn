#include <unwind.h>
#include "os/os.h"
#include "utils/Atomic.hxx"

#define MAX_STRACE 20

extern "C" {
extern void *stacktrace[MAX_STRACE];
extern int strace_len;
void call_unwind(void);
}

struct trace
{
    /// For quick comparison of traces.
    unsigned hash : 24;
    /// Number of entries in the trace.
    unsigned len : 8;
    struct trace *next;
    /// total memory (bytes) allocated via this trace.
    unsigned total_size;
};

struct trace *all_traces = nullptr;

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

extern "C" {
extern void *__wrap_malloc(size_t size);
extern void *__real_malloc(size_t size);
void* usb_malloc(unsigned long length);
}

struct trace *add_new_trace(unsigned hash)
{
    unsigned total_size = sizeof(struct trace) + strace_len * sizeof(stacktrace[0]);
#if defined(TARGET_LPC2368) || defined(TARGET_LPC1768)
    struct trace* t = (struct trace*)usb_malloc(total_size);
#else
    struct trace* t = (struct trace*)__real_malloc(total_size);
#endif
    memcpy(t + 1, stacktrace, strace_len * sizeof(stacktrace[0]));
    t->hash = hash;
    t->len = strace_len;
    t->total_size = 0;
    t->next = all_traces;
    all_traces = t;
    return t;
}
static Atomic* get_lock() {
    static Atomic lock;
    return &lock;
}

void *stacktrace[MAX_STRACE];
int strace_len;

_Unwind_Reason_Code trace_func(struct _Unwind_Context *context, void *arg)
{
    void *ip = (void *)_Unwind_GetIP(context);
    if (strace_len > 0 && stacktrace[strace_len - 1] == ip)
    {
        return _URC_END_OF_STACK;
    }
    if (strace_len >= MAX_STRACE)
    {
        return _URC_END_OF_STACK;
    }
    stacktrace[strace_len++] = ip;
    return _URC_NO_REASON;
}

void *__wrap_malloc(size_t size)
{
    unsigned saved_lr = 0;
#if defined(TARGET_LPC2368) || defined(TARGET_LPC1768)
    asm volatile ("mov %0, lr \n" : "=r" (saved_lr));
#endif
    {
        AtomicHolder holder(get_lock());
        strace_len = 0;
        _Unwind_Backtrace(&trace_func, 0);
        if (strace_len == 1) {
            stacktrace[strace_len++] = (void*)saved_lr;
        }
        unsigned h = hash_trace(strace_len, (unsigned *)stacktrace);
        struct trace *t = find_current_trace(h);
        if (!t)
        {
            t = add_new_trace(h);
        }
        t->total_size += size;
    }
    return __real_malloc(size);
}
