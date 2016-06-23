extern "C" {
#include "ets_sys.h"
#include <gpio.h>
#include <os_type.h>
#include <osapi.h>
#include <mem.h>
#include <user_interface.h>
#include "ets_rom.h"
}

#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>

#include "os/os.h"
#include "utils/blinker.h"
#include "hardware.hxx"

namespace nmranet {
extern char CONFIG_FILENAME[];
}

extern "C" {

extern void ets_delay_us(uint32_t us);

void resetblink(uint32_t pattern)
{
    if (pattern)
    {
        HW::BLINKER_RAW_Pin::set(!HW::blinker_invert);
    }
    else
    {
        HW::BLINKER_RAW_Pin::set(HW::blinker_invert);
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
            HW::BLINKER_RAW_Pin::set(!HW::blinker_invert);
        } else {
            HW::BLINKER_RAW_Pin::set(HW::blinker_invert);
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

extern void spiffs_init();

void init_done() {
    system_set_os_print(1);
    //gdb_init();
    do_global_ctors();
    spiffs_init();
    // Try to open the config file
    int fd = ::open(nmranet::CONFIG_FILENAME, O_RDONLY);
    if (fd < 0) fd = ::open(nmranet::CONFIG_FILENAME, O_CREAT|O_TRUNC|O_RDWR);
    if (fd < 0) {
        printf("Formatting the SPIFFS fs.");
        extern void esp_spiffs_deinit(uint8_t);
        esp_spiffs_deinit(1);
        spiffs_init();
    }

    appl_task(nullptr);
}

extern "C" void system_restart_local();

void reboot_now() {
    system_restart_local();
    //(*((volatile uint32_t*) 0x60000700)) |= 0x80000000;
}

void ICACHE_FLASH_ATTR user_init()
{
    gpio_init();
    HW::BLINKER_RAW_Pin::hw_init();
    HW::BLINKER_RAW_Pin::hw_set_to_safe();
    HW::GpioBootloaderInit::hw_init();

    //uart_div_modify(0, UART_CLK_FREQ / (74880));
    uart_div_modify(0, UART_CLK_FREQ / (115200));

    //abort();

    //uart_init(74880, 74880);

    os_printf("hello,world\n");
    // init gpio subsytem
    system_init_done_cb(&init_done);

    // static os_task_t appl_task_struct;
    // system_os_task(appl_task, USER_TASK_PRIO_0, appl_task_event, 1);

}


}  // extern "C"

void log_output(char* buf, int size) {
    if (size <= 0) return;
    printf("%s\n", buf);
}
