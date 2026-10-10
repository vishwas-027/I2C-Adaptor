#ifndef SYSTEM_H   /* Include guard start - avoids double-inclusion if system.h is pulled in from multiple translation units/headers. */
#define SYSTEM_H   /* Guard macro defined on first inclusion. */

#include <stdint.h>  /* Standard fixed-width integer types - gives us uint32_t, used for every clock-frequency value below (Hz values comfortably fit in 32 bits up to ~4.29GHz). */

/* CMSIS-compatible core frequency and the derived AHB/APB clock tree. */
extern uint32_t SystemCoreClock;
/* ^ This is NOT a hardware register - it's a plain RAM variable, declared
 *   `extern` here and DEFINED (with storage) in system.c. The name
 *   SystemCoreClock is a CMSIS convention: virtually every ARM Cortex-M
 *   vendor HAL/CMSIS pack exposes a global of exactly this name holding
 *   "the current CPU core clock in Hz". Anything that needs to compute
 *   timing (SysTick reload value, FreeRTOS's portNVIC_SYSTICK_* setup,
 *   blocking delay loops, UART baud-rate divisors) reads THIS variable
 *   rather than re-deriving the clock from RCC registers itself - it's the
 *   single source of truth for "how fast is the CPU running right now".
 *   Nothing updates it automatically; board.c's SystemCoreClockUpdate()
 *   call after every clock-tree change is what keeps it honest. If a clock
 *   switch ever happens without that call, every consumer of this
 *   variable will silently compute wrong timing. */

typedef struct
{
    uint32_t sysclk;  /* System clock (SYSCLK) in Hz - output of the clock mux (HSI/HSE/PLL), before any AHB prescaler is applied. */
    uint32_t hclk;    /* AHB bus clock (HCLK) in Hz - SYSCLK after the AHB prescaler (HPRE in RCC->CFGR). Drives the CPU core, DMA, and AHB peripherals. */
    uint32_t pclk1;   /* APB1 bus clock (PCLK1) in Hz - HCLK after the APB1 prescaler (PPRE1). Feeds I2C1, TIM2-5, USART2/3, SPI2/3, etc. This is the value your I2C1 timing-register (CCR/TRISE) math needs. */
    uint32_t pclk2;   /* APB2 bus clock (PCLK2) in Hz - HCLK after the APB2 prescaler (PPRE2). Feeds TIM1, USART1/6, SPI1, ADC, etc. */

} system_clock_t;
/* ^ Custom (non-CMSIS) struct that goes a step further than the single
 *   SystemCoreClock scalar: SystemCoreClock alone only ever tells you the
 *   CPU/AHB-side frequency, but firmware here also needs to know the two
 *   independent APB bus speeds (since they can be divided differently, as
 *   board.c does with PCLK1=/2 vs PCLK2=/1) - hence this struct bundling
 *   all four together instead of exposing four separate loose globals. */

extern system_clock_t SystemClock;
/* ^ The single global instance of the struct above - declared here,
 *   defined (with an initial reset-time value) in system.c. Anything that
 *   needs a bus clock other than the core clock (e.g. an I2C driver
 *   computing timing constants from PCLK1) reads SystemClock.pclk1 rather
 *   than trying to reconstruct it from SystemCoreClock by hand. */

/* Update all exported clock values after the board changes SYSCLK. */
void SystemCoreClockUpdate(uint32_t clock);
/* ^ The only function that's allowed to WRITE to SystemCoreClock/
 *   SystemClock - keeps all clock-tracking updates in one place instead of
 *   scattering direct assignments across board.c. MUST be called every
 *   single time the clock tree actually changes (see board.c's
 *   sysclk_switch_to_pll() and board_clock_init_debug_hsi()), or these
 *   globals go stale and every timing-dependent piece of code downstream
 *   silently computes against the wrong frequency. Takes the new SYSCLK/
 *   HCLK value in Hz as its one argument - see system.c for how the two
 *   APB clocks get derived from it. */

#endif  /* SYSTEM_H - include guard end. */