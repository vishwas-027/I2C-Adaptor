#include <stddef.h>
#include <stdint.h>

/* Symbols provided by the linker script */
extern uint32_t _sidata;  /* start of .data init values, in FLASH */
extern uint32_t _sdata;   /* start of .data, in RAM */
extern uint32_t _edata;   /* end of .data, in RAM */
extern uint32_t _sbss;    /* start of .bss */
extern uint32_t _ebss;    /* end of .bss, in RAM */
extern uint32_t _estack;  /* top of stack, computed in linker script */
extern uint32_t _end;     /* end of the .bss/.heap region from the linker script */

int main(void);

void Reset_Handler(void);
void Default_Handler(void);
void SystemInit(void);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void OTG_FS_IRQHandler(void);

/* Core Cortex-M4 fault/system handlers: weak aliases to Default_Handler
 * so the project links even before you implement real handlers. */
void NMI_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
// void SVC_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)   __attribute__((weak, alias("Default_Handler")));
// void PendSV_Handler(void)     __attribute__((weak, alias("Default_Handler")));
// void SysTick_Handler(void)    __attribute__((weak, alias("Default_Handler")));

extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

typedef void (*isr_handler)(void);

/* Total vector table size: 16 core entries (incl. initial SP) + 86 IRQs
 * for STM32F411. We only name the core handlers here; any peripheral
 * IRQ (USART2, TIM2, etc.) defaults to 0/unused since this demo polls
 * UART instead of using interrupts. Add [16 + IRQn] = YourHandler if you
 * enable an interrupt later (check the NVIC IRQ table in RM0383). */
#define VECTOR_TABLE_SIZE (16 + 86)

__attribute__((section(".isr_vector")))
const isr_handler vector_table[VECTOR_TABLE_SIZE] = {
    [0]  = (isr_handler)&_estack,
    [1]  = Reset_Handler,
    [2]  = NMI_Handler,
    [3]  = HardFault_Handler,
    [4]  = MemManage_Handler,
    [5]  = BusFault_Handler,
    [6]  = UsageFault_Handler,
    [11] = vPortSVCHandler,
    [12] = DebugMon_Handler,
    [14] = xPortPendSVHandler,
    [15] = xPortSysTickHandler,
    [16 + 31] = I2C1_EV_IRQHandler,
    [16 + 32] = I2C1_ER_IRQHandler,
    [16 + 67] = OTG_FS_IRQHandler,
};

void Reset_Handler(void)
{
    /* Copy .data initializers from FLASH to RAM */
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }

    /* Zero-fill .bss */
    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
    }

    SystemInit();
    
    main();

    /* main() should never return, but just in case */
    while (1) {
    }
}

void Default_Handler(void)
{
    while (1) {
        /* Unhandled interrupt/fault: sit here so you notice in the debugger */
    }
}

void *_sbrk(ptrdiff_t incr)
{
    static uint8_t *heap_end = NULL;
    uint8_t *prev_heap_end;

    if (heap_end == NULL) {
        heap_end = (uint8_t *)&_end;
    }

    prev_heap_end = heap_end;

    if ((uintptr_t)(heap_end + incr) > (uintptr_t)&_estack) {
        return (void *)-1;
    }

    heap_end += incr;
    return prev_heap_end;
}
