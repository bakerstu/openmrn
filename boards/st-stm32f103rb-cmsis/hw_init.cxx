#include "FreeRTOSConfig.h"
#include "os/OS.hxx"

#include "stm32f10x.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

extern "C" {

extern void Reset_Handler(void);
extern void __libc_init_array(void);
extern int main(void);
extern void SystemInit(void);

extern uint32_t __start_ram;
extern uint32_t __end_ram;

extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __data_init_start__;

extern uint32_t __bss_start__;
extern uint32_t __bss_end__;

void setup_blinker();
void setup_nvic();

void __attribute__((naked)) Reset_Handler(void) {
    // Takes care of hardware responses from the bootloader not showing up
    // anymore.
    __disable_irq();
    // Resets USB module
    RCC_APB2PeriphResetCmd(RCC_APB1Periph_USB, ENABLE);
    for (volatile int i = 0; i < 10; ++i);
    RCC_APB2PeriphResetCmd(RCC_APB1Periph_USB, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB1Periph_USB, DISABLE);

    // Fills the memory with a debug pattern.
    for (uint32_t* d = &__start_ram; d < &__end_ram; ++d)
    {
        *d = 0xdbdbdbdb;
    }

    // Clears BSS
    for (uint32_t* d = &__bss_start__; d < &__bss_end__; ++d) {
        *d = 0;
    }

    // Copies initialized data from flash to RAM
    uint32_t* s = &__data_init_start__;
    for (uint32_t* d = &__data_start__; d < &__data_end__; ++d, ++s) {
        *d = *s;
    }

    // Sets up system clock.
    SystemInit();

    setup_nvic();
    setup_blinker();

    // Calls global constructors.
    __libc_init_array();
    main();
    while(1);
}


void setup_nvic() {
    extern uint32_t g_pfnVectors;
    SCB->VTOR = (uint32_t)&g_pfnVectors;
    // 4 bits of interrupt priority, 0 bits for subpriority.
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    // Clears external and internal interrupt masks and software interrupt
    // request bits.
    EXTI->IMR = 0;
    EXTI->EMR = 0;
    EXTI->SWIER = 0;
    // Clears all NVIC interrupt enable and pending bits.
    for (unsigned i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); ++i) {
        NVIC->ICER[i] = NVIC_ICER_CLRENA;
        NVIC->ICPR[i] = NVIC_ICPR_CLRPEND;
    }
}

void setup_blinker() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gp;
    GPIO_StructInit(&gp);

    gp.GPIO_Pin = (1<<1) | (1<<5);
    gp.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &gp);

    GPIOA->BSRR = (1<<1) | (1<<5); // Turns on both leds.
}

void setblink(uint32_t pattern);

const unsigned long cm3_cpu_clock_hz = 72000000;

void hw_init(void)
{
    // This is needed for proper bootup of the FreeRTOS scheduler.
    __disable_irq();
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
    __enable_irq();
}

void diewith(unsigned long);

void Default_Handler() {
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
    GPIOA->BSRR = pattern ? (1<<17) : (1<<1);
    //gpio_write_bit(ERROR_LED_PORT, ERROR_LED_PIN, );
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
    __disable_irq();
    while(1) {
        GPIOA->BSRR = current_pattern & 1 ? ((1<<5)|(1<<1)) : ((1<<5)|(1<<1)) << 16;
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
void HardFault_Handler(void)
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

}  // extern C
