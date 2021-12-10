#include <string.h>

#define BOOTLOADER_STREAM
//#define BOOTLOADER_DATAGRAM

#include "BootloaderHal.hxx"
#include "bootloader_hal.h"

#include "nmranet_config.h"
#include "openlcb/Defs.hxx"
#include "Stm32Gpio.hxx"
#include "openlcb/Bootloader.hxx"
#include "openlcb/If.hxx"
#include "utils/GpioInitializer.hxx"

const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};
const uint32_t HSEValue = 8000000;

int g_death_lineno = 0;

extern "C" {

GPIO_PIN(LED_GREEN, LedPin, A, 5);
GPIO_PIN(SW1, GpioInputPU, C, 13);

static constexpr unsigned clock_hz = 48000000;

void bootloader_hw_set_to_safe(void)
{
    SW1_Pin::hw_set_to_safe();
    LED_GREEN_Pin::hw_set_to_safe();
}

extern void bootloader_reset_segments(void);
//extern unsigned long cm3_cpu_clock_hz;

/** Setup the system clock */
static void clock_setup(void)
{
    /* reset clock configuration to default state */
    RCC->CR = RCC_CR_HSITRIM_4 | RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY))
        ;

#define USE_EXTERNAL_8_MHz_CLOCK_SOURCE 1
/* configure PLL:  8 MHz * 6 = 48 MHz */
#if USE_EXTERNAL_8_MHz_CLOCK_SOURCE
    RCC->CR |= RCC_CR_HSEON | RCC_CR_HSEBYP;
    while (!(RCC->CR & RCC_CR_HSERDY))
        ;
    RCC->CFGR = RCC_CFGR_PLLMUL6 | RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_SW_HSE;
    while (!((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSE))
        ;
#else
    RCC->CFGR = RCC_CFGR_PLLMUL6 | RCC_CFGR_PLLSRC_HSI_PREDIV | RCC_CFGR_SW_HSI;
    while (!((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSI))
        ;
#endif
    /* enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    /* set PLL as system clock */
    RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_PLL;
    while (!((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL))
        ;
}

void bootloader_hw_init()
{
    /* Globally disables interrupts until the FreeRTOS scheduler is up. */
    asm("cpsid i\n");

    /* these FLASH settings enable opertion at 48 MHz */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

    /* Reset HSI14 bit */
    RCC->CR2 &= (uint32_t)0xFFFFFFFEU;

    /* Disable all interrupts */
    RCC->CIR = 0x00000000U;

    clock_setup();

    /* enable peripheral clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();

    /* setup pinmux */
    GPIO_InitTypeDef gpio_init;
    memset(&gpio_init, 0, sizeof(gpio_init));

    /* CAN pinmux on PB8 and PB9 */
    gpio_init.Mode = GPIO_MODE_AF_PP;
    // Disables pull-ups because this is a 5V tolerant pin.
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF4_CAN;
    gpio_init.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOB, &gpio_init);
    gpio_init.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    LED_GREEN_Pin::hw_init();
    SW1_Pin::hw_init();

    /* disable sleep, enter init mode */
    CAN->MCR = CAN_MCR_INRQ;

    /* Time triggered tranmission off
     * Bus off state is left automatically
     * Auto-Wakeup mode disabled
     * automatic re-transmission enabled
     * receive FIFO not locked on overrun
     * TX FIFO mode on
     */
    CAN->MCR |= (CAN_MCR_ABOM | CAN_MCR_TXFP);

    /* Setup timing.
     * 125,000 Kbps = 8 usec/bit
     */
    CAN->BTR = (CAN_BS1_5TQ | CAN_BS2_2TQ | CAN_SJW_1TQ |
                ((clock_hz / 1000000) - 1));

    /* enter normal mode */
    CAN->MCR &= ~CAN_MCR_INRQ;

    /* Enter filter initialization mode.  Filter 0 will be used as a single
     * 32-bit filter, ID Mask Mode, we accept everything, no mask.
     */
    CAN->FMR |= CAN_FMR_FINIT;
    CAN->FM1R = 0;
    CAN->FS1R = 0x000000001;
    CAN->FFA1R = 0;
    CAN->sFilterRegister[0].FR1 = 0;
    CAN->sFilterRegister[0].FR2 = 0;

    /* Activeate filter and exit initialization mode. */
    CAN->FA1R = 0x000000001;
    CAN->FMR &= ~CAN_FMR_FINIT;
}

void bootloader_led(enum BootloaderLed id, bool value)
{
    switch(id)
    {
        case LED_ACTIVE:
            LED_GREEN_Pin::set(value);
            return;
        case LED_WRITING:
            LED_GREEN_Pin::set(value);
            return;
        case LED_CSUM_ERROR:
            return;
        case LED_REQUEST:
            return;
        case LED_FRAME_LOST:
            return;
        default:
            /* ignore */
            break;
    }
}

bool request_bootloader()
{
    extern uint32_t __bootloader_magic_ptr;
    if (__bootloader_magic_ptr == REQUEST_BOOTLOADER) {
        __bootloader_magic_ptr = 0;
        LED_GREEN_Pin::set(true);
        return true;
    }
    LED_GREEN_Pin::set(SW1_Pin::get());
    return !SW1_Pin::get();
}

} // extern "C"
