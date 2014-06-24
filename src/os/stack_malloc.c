#include <stdlib.h>

#if defined (__FreeRTOS__)
const void* __attribute__((weak)) stack_malloc(unsigned long length);


const void* stack_malloc(unsigned long length)
{
    /* We do a trick here to ensure that the compiler will output a stack frame
     * for this function. We want to avoid tail-chain optimization in this
     * function or else it disappears from the stack traces done for memory
     * tracing. */
    void* volatile v = malloc(length);
    return v;
}
#endif
