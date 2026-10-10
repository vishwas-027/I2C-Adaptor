#include "../include/stm32f411.h"

void SystemInit(void)
{
    /* Enable the Cortex-M4F floating-point unit before any task or library
     * code can issue a floating-point instruction. */
    SCB->CPACR |= (0xFU << 20);
}
