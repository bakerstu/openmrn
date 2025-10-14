#include <stdlib.h>

void *bget_impl_malloc(size_t size);
void bget_impl_free(void *ptr);
void *bget_impl_calloc(size_t size);
void *bget_impl_realloc(void *ptr, size_t size);
