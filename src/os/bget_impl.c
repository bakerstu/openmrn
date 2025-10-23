#include "os/bget_impl.h"
#include "os/bget.h"

extern void* _sbrk_r(struct _reent *reent, ptrdiff_t incr);

/// Must be the same size as SizeQuant in bget.c
#define SIZE_QUANT 16

/// Add more system memory to BGET.
/// @param incr (minimum) size in bytes to allocate. Additional metadata
///        padding will be added.
static void bget_impl_add(size_t incr)
{
    incr = (incr + (SIZE_QUANT - 1)) & (~(SIZE_QUANT - 1));
    incr += 4 * sizeof(bufsize); // Header block.
    void *mem = _sbrk_r(NULL, incr);
    bpool(mem, incr);
}

//
// bget_impl_malloc()
//
void *bget_impl_malloc(size_t size)
{
    void *result = bget(size);
    if (result == NULL)
    {
        bget_impl_add(size);
        result = bget(size);
    }
    return result;
}

//
// bget_impl_calloc()
//
void *bget_impl_calloc(size_t size)
{
    void *result = bgetz(size);
    if (result == NULL)
    {
        bget_impl_add(size);
        result = bgetz(size);
    }
    return result;
}

//
// bget_impl_realloc()
//
void *bget_impl_realloc(void *ptr, size_t size)
{
    void *result = bgetr(ptr, size);
    if (result == NULL)
    {
        bget_impl_add(size);
        result = bgetr(ptr, size);
    }
    return result;
}

//
// bget_impl_free()
//
void bget_impl_free(void *ptr)
{
    if (ptr)
    {
        brel(ptr);
    }
}