#include <stdlib.h>

/// Invoke BGET implementation of malloc().
/// @param size size of item in bytes to allocated
/// @return allocated memory buffer, else nullptr if out of heap memory
void *bget_impl_malloc(size_t size);

/// Invoke BGET implementation of calloc().
/// @param size size of item in bytes to allocated
/// @return allocated memory buffer, zero initialized, else nullptr if out of
///         heap memory
void *bget_impl_calloc(size_t size);

/// Invoke BGET implementation of realloc().
/// @param size new size of item in bytes to allocated
/// @return allocated memory buffer with previous allocation data copied over,
///         else nullptr if out of heap memory
void *bget_impl_realloc(void *ptr, size_t size);

/// Invoke BGET implementation of free().
/// @param ptr pointer to the buffer to free
void bget_impl_free(void *ptr);