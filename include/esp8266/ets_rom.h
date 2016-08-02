#ifndef _ETS_ROM_H
#define _ETS_ROM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ets_sys.h"
#include <osapi.h>
#include <os_type.h>
#include <c_types.h>

#include <os_type.h>
#include <stdarg.h>

typedef void (*int_handler_t)(void*);

void *pvPortMalloc(size_t xWantedSize, const char* file, int line) __attribute__((malloc, alloc_size(1)));
void *pvPortRealloc(void* ptr, size_t xWantedSize, const char* file, int line) __attribute__((alloc_size(2)));
void vPortFree(void *ptr, const char* file, int line);
void *ets_memcpy(void *dest, const void *src, size_t n);
void *ets_memset(void *s, int c, size_t n);
void ets_timer_arm_new(ETSTimer *a, int b, int c, int isMstimer);
void ets_timer_setfn(ETSTimer *t, ETSTimerFunc *fn, void *parg);
void ets_timer_disarm(ETSTimer *a);
int atoi(const char *nptr);
int ets_strncmp(const char *s1, const char *s2, int len);
int ets_strcmp(const char *s1, const char *s2);
int ets_strlen(const char *s);
char *ets_strcpy(char *dest, const char *src);
char *ets_strncpy(char *dest, const char *src, size_t n);
char *ets_strstr(const char *haystack, const char *needle);
int ets_sprintf(char *str, const char *format, ...)  __attribute__ ((format (printf, 2, 3)));
int ets_snprintf(char *str, size_t size, const char *format, ...) __attribute__ ((format (printf, 3, 4)));
int ets_printf(const char *format, ...)  __attribute__ ((format (printf, 1, 2)));
void ets_install_putc1(void* routine);
void uart_div_modify(int no, int freq);
void ets_isr_mask(int intr);
void ets_isr_unmask(int intr);
void ets_isr_attach(int intr, int_handler_t handler, void *arg);
void ets_intr_lock();
void ets_intr_unlock();
int ets_vsnprintf(char * s, size_t n, const char * format, va_list arg)  __attribute__ ((format (printf, 3, 0)));
int ets_vprintf(int (*print_function)(int), const char * format, va_list arg) __attribute__ ((format (printf, 2, 0)));
int ets_putc(int);
bool ets_task(ETSTask task, uint8 prio, ETSEvent *queue, uint8 qlen);
bool ets_post(uint8 prio, ETSSignal sig, ETSParam par);


#ifdef __cplusplus
} // extern "C"
#endif

#endif // _ETS_ROM_H
