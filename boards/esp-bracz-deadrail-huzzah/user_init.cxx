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
#include "freertos/bootloader_hal.h"

namespace openlcb {
extern char CONFIG_FILENAME[];
}

extern "C" {

void gpio16_output_conf(void)
{
    WRITE_PERI_REG(PAD_XPD_DCDC_CONF,
                   (READ_PERI_REG(PAD_XPD_DCDC_CONF) & 0xffffffbc) | (uint32)0x1); 	// mux configuration for XPD_DCDC to output rtc_gpio0

    WRITE_PERI_REG(RTC_GPIO_CONF,
                   (READ_PERI_REG(RTC_GPIO_CONF) & (uint32)0xfffffffe) | (uint32)0x0);	//mux configuration for out enable

    WRITE_PERI_REG(RTC_GPIO_ENABLE,
                   (READ_PERI_REG(RTC_GPIO_ENABLE) & (uint32)0xfffffffe) | (uint32)0x1);	//out enable
}

void gpio16_output_set(uint8 value)
{
    WRITE_PERI_REG(RTC_GPIO_OUT,
                   (READ_PERI_REG(RTC_GPIO_OUT) & (uint32)0xfffffffe) | (uint32)(value & 1));
}

void gpio16_input_conf(void)
{
    WRITE_PERI_REG(PAD_XPD_DCDC_CONF,
                   (READ_PERI_REG(PAD_XPD_DCDC_CONF) & 0xffffffbc) | (uint32)0x1); 	// mux configuration for XPD_DCDC and rtc_gpio0 connection

    WRITE_PERI_REG(RTC_GPIO_CONF,
                   (READ_PERI_REG(RTC_GPIO_CONF) & (uint32)0xfffffffe) | (uint32)0x0);	//mux configuration for out enable

    WRITE_PERI_REG(RTC_GPIO_ENABLE,
                   READ_PERI_REG(RTC_GPIO_ENABLE) & (uint32)0xfffffffe);	//out disable
}

uint8 gpio16_input_get(void)
{
    return (uint8)(READ_PERI_REG(RTC_GPIO_IN_DATA) & 1);
}

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
    int fd = ::open(openlcb::CONFIG_FILENAME, O_RDONLY);
    if (fd < 0) fd = ::open(openlcb::CONFIG_FILENAME, O_CREAT|O_TRUNC|O_RDWR);
    if (fd < 0) {
        printf("Formatting the SPIFFS fs.");
        extern void esp_spiffs_deinit(uint8_t);
        esp_spiffs_deinit(1);
        spiffs_init();
    }

    printf("userinit pinout: B hi %d; B lo %d; A hi %d; A lo %d;\n",
           HW::MOT_B_HI_Pin::PIN, 
           HW::MOT_B_LO_Pin::PIN, 
           HW::MOT_A_HI_Pin::PIN, 
           HW::MOT_A_LO_Pin::PIN);
    appl_task(nullptr);
}

extern "C" void system_restart_local();

void reboot_now() {
    system_restart_local();
}

void reboot() {
    system_restart_local();
}

void enter_bootloader() {
    extern uint32_t __bootloader_magic_ptr;
    __bootloader_magic_ptr = REQUEST_BOOTLOADER;
    
    system_restart_local();
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
