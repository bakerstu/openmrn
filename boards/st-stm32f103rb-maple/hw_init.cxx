#include "FreeRTOSConfig.h"
#include "os/OS.hxx"

#include <wirish/wirish.h>

extern "C" {

void setblink(uint32_t pattern);

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain()
{
    init();
    setblink(0);

    /* Set up the LED to blink  */
    gpio_set_mode(GPIOA, 5, GPIO_MODE_OUTPUT);
    gpio_set_mode(GPIOA, 1, GPIO_MODE_OUTPUT);
    gpio_write_bit(GPIOA, 5, 0);
    gpio_write_bit(GPIOA, 1, 0);

    while(0) {
        gpio_write_bit(GPIOA, 5, 0);
        gpio_write_bit(GPIOA, 1, 1);
        for (volatile int i = 0; i < 300000; i++);
        gpio_write_bit(GPIOA, 5, 1);
        gpio_write_bit(GPIOA, 1, 0);
        for (volatile int i = 0; i < 300000; i++);
    }

}

void hw_init(void)
{
    // This is needed for proper bootup of the FreeRTOS scheduler.
    nvic_globalirq_disable();
}

uint32_t blinker_pattern;
uint32_t current_pattern;
#define BLINK_GPIO GPIOA
#define BLINK_BIT 5

extern "C" {
extern void vPortClearInterruptMask( unsigned long ulNewMaskValue );
extern unsigned long ulPortSetInterruptMask( void );
}

void enable_fiq_only(void)
{
    /** @TODO(balazs.racz): we should call vPortClearInterruptMask(14) here,
     * assuming the blink-timer interrupt is set to priority 15. The code here
     * will block all FreeRTOS functions though. */
    ulPortSetInterruptMask();
    nvic_globalirq_enable();
}

void diewith(unsigned long);

// Used by libmaple
void _fail(const char* file, int line, const char* exp) {
    diewith(0x80CCACCA);
}

void default_interrupt_handler() {
    diewith(BLINK_DIE_UNEXPIRQ);
}

/** Initializes the timer responsible for the blinking hardware and sets
 *   initial pattern.
 *
 *  \param pattern is a blinking pattern, each bit will be shifted to the
 *  output LED every 125 ms.
 */
void setblink(uint32_t pattern)
{
    gpio_write_bit(ERROR_LED_PORT, ERROR_LED_PIN, pattern ? 1 : 0);
/*    blinker_pattern = pattern;
    current_pattern = 0;
    BLINK_GPIO->FIODIR |= BLINK_BIT;
    BLINK_GPIO->FIOCLR = BLINK_BIT;

    // clock = raw clock
    LPC_SC->PCLKSEL0 = (LPC_SC->PCLKSEL0 & (~(0x3 << 4))) | (0x01 << 4);
    LPC_TIM1->TCR = 2;  // stop & reset timer
    LPC_TIM1->CTCR = 0; // timer mode
    // prescale to 1 ms per tick
    LPC_TIM1->PR = configCPU_CLOCK_HZ / 1000;
    LPC_TIM1->MR0 = 125;
    LPC_TIM1->MCR = 3; // reset and interrupt on match 0

    NVIC_SetPriority(TIMER1_IRQn, 0);
    NVIC_EnableIRQ(TIMER1_IRQn);

    LPC_TIM1->TCR = 1; // Timer go.*/
}

/** Updates the blinking pattern.
 *
 *  \param pattern is a blinking pattern, each bit will be shifted to the
 *  output LED every 125 ms.
 */
void resetblink(uint32_t pattern)
{
    blinker_pattern = pattern;
    setblink(pattern);
    // Makes a timer event trigger immediately.
    //LPC_TIM1->TC = LPC_TIM1->MR0 - 2;
}

/** Aborts the program execution and sets a particular blinking pattern.
 *
 *  \param pattern is a blinking pattern, each bit will be shifted to the
 *  output LED every 125 ms.
 *
 *  Never returns.
 */
void diewith(unsigned long pattern)
{
    nvic_globalirq_disable();
    while(1) {
        gpio_write_bit(GPIOA, 5, current_pattern & 1 ? 1 : 0);
        gpio_write_bit(GPIOA, 1, current_pattern & 1 ? 1 : 0);
        current_pattern >>= 1;
        if (!current_pattern)
        {
            current_pattern = pattern;
        }
        for (volatile int i = 0; i < 450000; i++);
    }

    enable_fiq_only();
    setblink(pattern);
    for (;;)
    {
    }
}

void TIMER1_IRQHandler(void)
{
    if (current_pattern & 1)
    {
//        BLINK_GPIO->FIOSET = BLINK_BIT;
    }
    else
    {
//     BLINK_GPIO->FIOCLR = BLINK_BIT;
    }
    current_pattern >>= 1;
    if (!current_pattern)
    {
        current_pattern = blinker_pattern;
    }

    // Clears IRQ.
//    LPC_TIM1->IR = 1;
//    NVIC_ClearPendingIRQ(TIMER1_IRQn);
}

__attribute__((__naked__)) void hard_fault_handler_c( unsigned long *hardfault_args )
{
    /* These are volatile to try and prevent the compiler/linker optimising them
    away as the variables never actually get used.  If the debugger won't show the
    values of the variables, make them global my moving their declaration outside
    of this function. */
    volatile unsigned long stacked_r0 ;
    volatile unsigned long stacked_r1 ;
    volatile unsigned long stacked_r2 ;
    volatile unsigned long stacked_r3 ;
    volatile unsigned long stacked_r12 ;
    volatile unsigned long stacked_lr ;
    volatile unsigned long stacked_pc ;
    volatile unsigned long stacked_psr ;
    volatile unsigned long _CFSR ;
    volatile unsigned long _HFSR ;
    volatile unsigned long _DFSR ;
    volatile unsigned long _AFSR ;
    volatile unsigned long _BFAR ;
    volatile unsigned long _MMAR ;

    stacked_r0 = ((unsigned long)hardfault_args[0]) ;
    stacked_r1 = ((unsigned long)hardfault_args[1]) ;
    stacked_r2 = ((unsigned long)hardfault_args[2]) ;
    stacked_r3 = ((unsigned long)hardfault_args[3]) ;
    stacked_r12 = ((unsigned long)hardfault_args[4]) ;
    stacked_lr = ((unsigned long)hardfault_args[5]) ;
    stacked_pc = ((unsigned long)hardfault_args[6]) ;
    stacked_psr = ((unsigned long)hardfault_args[7]) ;

    // Configurable Fault Status Register
    // Consists of MMSR, BFSR and UFSR
    _CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;   
                                                                                    
    // Hard Fault Status Register
    _HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;

    // Debug Fault Status Register
    _DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;

    // Auxiliary Fault Status Register
    _AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;

    // Read the Fault Address Registers. These may not contain valid values.
    // Check BFARVALID/MMARVALID to see if they are valid values
    // MemManage Fault Address Register
    _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
    // Bus Fault Address Register
    _BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;

    __asm("BKPT #0\n") ; // Break into the debugger

    /* When the following line is hit, the variables contain the register values. */
    if (stacked_r0 || stacked_r1 || stacked_r2 || stacked_r3 || stacked_r12 ||
        stacked_lr || stacked_pc || stacked_psr || _CFSR || _HFSR || _DFSR ||
        _AFSR || _MMAR || _BFAR)
    {
        resetblink(BLINK_DIE_HARDFAULT);
        for( ;; );
    }
}

/** The fault handler implementation.  This code is inspired by FreeRTOS.
 */
static void hard_fault_handler(void)
{
    __asm volatile
    (
        "BKPT  #0 \n"
        "MOVS   R0, #4                  \n"
        "MOV    R1, LR                  \n"
        "TST    R0, R1                  \n"
        "BEQ    _MSP                    \n"
        "MRS    R0, PSP                 \n"
        "B      hard_fault_handler_c    \n"
        "_MSP:                          \n"
        "MRS    R0, MSP                 \n"
        "B      hard_fault_handler_c    \n"
        "BX    LR\n"
    );
}

void __exc_nmi(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __exc_hardfault(void) __attribute__ ((weak, alias ("hard_fault_handler")));
void __exc_memmanage(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __exc_busfault(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __exc_usagefault(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __stm32reservedexception7(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __stm32reservedexception8(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __stm32reservedexception9(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __stm32reservedexception10(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __exc_debug_monitor(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __stm32reservedexception13(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_wwdg(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_pvd(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_tamper(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_rtc(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_flash(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_rcc(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_exti0(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_exti1(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_exti2(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_exti3(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_exti4(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_dma1_channel1(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_dma1_channel2(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_dma1_channel3(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_dma1_channel4(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_dma1_channel5(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_dma1_channel6(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_dma1_channel7(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_adc(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_usb_hp_can_tx(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_usb_lp_can_rx0(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_can_rx1(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_can_sce(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_exti9_5(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_tim1_brk(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_tim1_up(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_tim1_trg_com(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_tim1_cc(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_tim2(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_tim3(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_tim4(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_i2c1_ev(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_i2c1_er(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_i2c2_ev(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_i2c2_er(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_spi1(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_spi2(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_usart1(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_usart2(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_usart3(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_exti15_10(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_rtcalarm(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_usbwakeup(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_tim8_brk(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_tim8_up(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_tim8_trg_com(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_tim8_cc(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_adc3(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_fsmc(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_sdio(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_tim5(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_spi3(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_uart4(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_uart5(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_tim6(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_tim7(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_dma2_channel1(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_dma2_channel2(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_dma2_channel3(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void __irq_dma2_channel4_5(void) __attribute__ ((weak, alias ("default_interrupt_handler")));

}  // extern C
