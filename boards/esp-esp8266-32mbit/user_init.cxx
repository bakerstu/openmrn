extern "C" {
#include "ets_sys.h"
#include <gpio.h>
#include <os_type.h>
#include <osapi.h>
#include <mem.h>
#include <user_interface.h>
#include "ets_rom.h"
}

#include "os/os.h"

#include "utils/blinker.h"

static int blinkerpin = 0;

extern "C" {

extern void ets_delay_us(uint32_t us);

void resetblink(uint32_t pattern)
{
    if (pattern)
    {
        gpio_output_set(0, (1 << blinkerpin), 0, 0);
    }
    else
    {
        gpio_output_set((1 << blinkerpin), 0, 0, 0);
    }
}

void setblink(uint32_t pattern)
{
    resetblink(pattern);
}

void __attribute__((noreturn)) diewith(uint32_t pattern)
{
    extern void ets_wdt_disable();
    ets_wdt_disable();
    uint32_t p = 0;
    while(true) {
        if (p & 1) {
            gpio_output_set(0, (1 << blinkerpin), 0, 0);
        } else {
            gpio_output_set((1 << blinkerpin), 0, 0, 0);
        }
        p >>= 1;
        if (!p) p = pattern;
        ets_delay_us(125000);
    }
    /*
    resetblink(pattern);
    while (1)
    ;*/
}

void ICACHE_FLASH_ATTR abort() {
    diewith(BLINK_DIE_ABORT);
}

void ICACHE_FLASH_ATTR usleep(useconds_t sleep_usec)
{
    ets_delay_us(sleep_usec);
}

//static os_event_t appl_task_event[1];

void ICACHE_FLASH_ATTR appl_task(os_event_t *e)
{
    static int argc = 0;
    static char **argv = {0};
    appl_main(argc, argv);
}

struct _reent* _impure_ptr = nullptr;

char noerror[] = "strerror output";

char* strerror(int) {
    return noerror;
}

extern void (*__init_array_start)(void);
extern void (*__init_array_end)(void);

static void do_global_ctors(void) {
    void (**p)(void);
    for(p = &__init_array_start; p != &__init_array_end; ++p)
        (*p)();
}



void init_done() {
    system_set_os_print(1);
    //gdb_init();
    do_global_ctors();
    appl_task(nullptr);
}



void ICACHE_FLASH_ATTR user_init()
{
    gpio_init();
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);
    gpio_output_set(0, 0, (1 << blinkerpin), 0);

    uart_div_modify(0, UART_CLK_FREQ / (74880));

    //abort();

    //uart_init(74880, 74880);

    os_printf("hello,world\n");
    // init gpio subsytem
    system_init_done_cb(&init_done);

    // static os_task_t appl_task_struct;
    // system_os_task(appl_task, USER_TASK_PRIO_0, appl_task_event, 1);

}


}  // extern "C"
