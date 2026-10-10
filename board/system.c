#include "system.h"
/* Brings in the declarations (extern SystemCoreClock, extern SystemClock,
 * the system_clock_t typedef, and the SystemCoreClockUpdate() prototype)
 * that this file provides the actual storage/implementation for. */

uint32_t SystemCoreClock = 16000000U;
/* ^ Actual storage for the CMSIS-style global (system.h only `extern`
 *   declared it - this is where the linker actually allocates the 4 bytes).
 *   Initialized to 16,000,000 (16MHz) because that's what the chip is
 *   ACTUALLY running at from reset, before board_init()/board_clock_init()
 *   ever executes: the F411 always boots on the internal HSI RC oscillator
 *   at 16MHz with no PLL, no wait states configured, nothing. This value
 *   is a reset-time snapshot, not a guess - it must match silicon reality
 *   at power-up or any code that runs before board_clock_init() (early
 *   startup code, very first UART prints, etc.) would compute wrong
 *   timing. */

system_clock_t SystemClock =
{
    .sysclk = 16000000U,  /* Matches SystemCoreClock above - SYSCLK = HSI = 16MHz at reset, no PLL involved yet. */
    .hclk   = 16000000U,  /* AHB prescaler (HPRE) is /1 at reset (hardware default), so HCLK = SYSCLK = 16MHz. */
    .pclk1  = 16000000U,  /* APB1 prescaler (PPRE1) is /1 at reset (hardware default - NOT yet divided by 2; that /2 only gets programmed once board_clock_init()/board_clock_init_debug_hsi() runs), so PCLK1 = HCLK = 16MHz here. */
    .pclk2  = 16000000U   /* APB2 prescaler (PPRE2) is /1 at reset too, so PCLK2 = HCLK = 16MHz. */
};
/* ^ Reset-time initializer for the bundled clock-tree struct. Note this is
 *   only accurate for the brief window between chip reset and whichever
 *   board_clock_init*() function runs - the moment board.c reprograms the
 *   RCC prescalers, these numbers must be refreshed via
 *   SystemCoreClockUpdate() below, or they'll describe a clock tree that
 *   no longer exists in hardware. */

/*
 * SystemCoreClockUpdate()
 * ------------------------
 * Called by board.c immediately after any actual hardware clock-tree
 * change (PLL switch-in, HSI switch-back, etc.) to keep these tracking
 * globals in sync with reality. This function does NOT touch any RCC
 * register itself - it's pure bookkeeping, recomputing "what should the
 * bus clocks be" from the single new core-clock value it's handed.
 */
void SystemCoreClockUpdate(uint32_t clock)
{
    SystemCoreClock = clock;
    /* Store the new core/AHB clock directly - this is exactly the value
     * the caller (board.c) just configured HCLK to be. */

    SystemClock.sysclk = clock;
    /* SYSCLK == the value passed in. Correct as long as the caller always
     * passes the post-switch SYSCLK value (which board.c does: 84000000U
     * or 16000000U to match whichever path just ran). */

    SystemClock.hclk   = clock;
    /* HCLK == SYSCLK here because both board_clock_init() and
     * board_clock_init_debug_hsi() always leave the AHB prescaler (HPRE)
     * at /1 - this function doesn't independently know the HPRE value, it
     * just HARD-CODES the assumption that HCLK always equals SYSCLK. If a
     * future clock-tree change ever divides HCLK below SYSCLK, this line
     * would need to change too - it isn't derived from any register read,
     * it's baked in. */

    SystemClock.pclk1  = clock / 2U;
    /* PCLK1 = clock / 2 - this HARD-CODES the assumption that APB1's
     * prescaler (PPRE1) is always programmed to /2, which matches what
     * board.c actually does in both sysclk_switch_to_pll() (84MHz -> 42MHz)
     * and board_clock_init_debug_hsi() (16MHz -> 8MHz). This is the exact
     * "fixed assumption" referenced in board.c's debug-path comments.
     * IMPORTANT CAVEAT: this function has no way of knowing the real
     * PPRE1 bits in RCC->CFGR - if board.c's prescaler configuration ever
     * changes (e.g. someone sets PPRE1_DIV4 instead of DIV2, or changes it
     * conditionally), this line must be updated to match by hand, or
     * SystemClock.pclk1 will silently report the wrong value while the
     * real hardware runs at a different PCLK1 - exactly the kind of
     * mismatch worth double-checking given the I2C1 timing-register math
     * that depends on this number. */

    SystemClock.pclk2  = clock;
    /* PCLK2 = clock (no division) - hard-codes the assumption that APB2's
     * prescaler (PPRE2) is always /1, which again matches what board.c
     * actually programs (PPRE2 field cleared and left at reset default in
     * both clock paths). Same caveat as pclk1 above: this is asserted, not
     * read back from hardware. */
}