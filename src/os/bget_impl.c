#include "os/bget_impl.h"
#include "os/bget.h"

extern char __heap2_start_alias;
extern char __heap2_start;
extern char __heap2_end;

extern char *heap_end;
extern char *heap2_end;

static void bget_impl_add(size_t incr)
{
    incr += 128;
    incr = (incr + (256U - 1U)) & ~(256U - 1U);

    /** @todo (Stuart Baker) change naming to remove "cs3" convention */
    extern char __cs3_heap_start;
    extern char __cs3_heap_end; /* Defined by the linker */
    if (heap_end == 0)
    {
        heap_end = &__cs3_heap_start;
    }
    if (heap2_end == 0)
    {
        heap2_end = &__heap2_start;
    }
    if ((heap_end + incr) > &__cs3_heap_end)
    {
        if (&__heap2_start != &__heap2_end)
        {
            /* there is a second heap */
            if ((heap2_end + incr) <= &__heap2_end)
            {
                bpool(heap2_end, incr);
                heap2_end += incr;
            }
        }
        /* Heap and stack collision */
        return;
    }
    bpool(heap_end, incr);
    heap_end += incr;
}

void *bget_impl_malloc(size_t size)
{
    void *result = NULL;
    result = bget(size);
    if (result == NULL)
    {
        bget_impl_add(size);
        result = bget(size);
    }
    return result;
}

void bget_impl_free(void *ptr)
{
    if (ptr)
    {
        brel(ptr);
    }
}