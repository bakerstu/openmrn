#include <stdlib.h>

#if defined (__FreeRTOS__)
const void* __attribute__((weak)) stack_malloc(unsigned long length)
{
    return malloc(length);
}
#endif
