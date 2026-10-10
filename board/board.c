#include "../include/stm32f411.h"
/* ^ Your custom register-definition header: gives us the FLASH_TypeDef and
 *   RCC_TypeDef peripheral structs (FLASH->, RCC->) plus every bit-position
 *   and mask macro used below (RCC_CR_PLLON, FLASH_ACR_LATENCY_2WS, etc). */
#include "board.h"
/* ^ Declares the public prototypes (boardInit, boardClockInit,
 *   boardClockInit_debug_hsi) that this file implements. */
#include "system.h"
/* ^ Declares SystemCoreClockUpdate() - lets us tell the rest of the
 *   firmware (FreeRTOS tick config, UART baud-rate math, software delay
 *   loops) what the core clock actually is after we change it. Nothing
 *   automatically re-reads the clock tree, so every place that changes
 *   SYSCLK must call this or downstream timing breaks silently. */

/* Official WeAct STM32F411 Black Pill boards use a 25 MHz crystal. The
 * divider values below retain the existing 84 MHz system clock and produce
 * the accurate 48 MHz PLLQ clock required by USB OTG FS:
 *
 *   PLL input = 25 MHz / 25 = 1 MHz
 *   PLL VCO   = 1 MHz * 336 = 336 MHz
 *   SYSCLK    = 336 MHz / 4 = 84 MHz
 *   USB clock = 336 MHz / 7 = 48 MHz
 */
#define BOARD_HSE_CLOCK_HZ 25000000U
#define BOARD_PLL_M         25U
#define BOARD_PLL_N         336U
#define BOARD_PLL_P_BITS    1U  /* PLLP encoding 1 selects divide-by-4. */
#define BOARD_PLL_P_DIVISOR 4U
#define BOARD_PLL_Q         7U
#define BOARD_PLL_INPUT_HZ  (BOARD_HSE_CLOCK_HZ / BOARD_PLL_M)
#define BOARD_PLL_VCO_HZ    (BOARD_PLL_INPUT_HZ * BOARD_PLL_N)
#define BOARD_SYSCLK_HZ     (BOARD_PLL_VCO_HZ / BOARD_PLL_P_DIVISOR)
#define BOARD_USB_CLOCK_HZ  (BOARD_PLL_VCO_HZ / BOARD_PLL_Q)

_Static_assert((BOARD_HSE_CLOCK_HZ % BOARD_PLL_M) == 0U,
               "HSE must divide evenly into the PLL input clock");
_Static_assert(BOARD_PLL_INPUT_HZ == 1000000U,
               "PLL input must remain at 1 MHz");
_Static_assert(BOARD_SYSCLK_HZ == 84000000U,
               "System clock must remain at 84 MHz");
_Static_assert(BOARD_USB_CLOCK_HZ == 48000000U,
               "USB OTG FS requires a 48 MHz PLLQ clock");

/* Start the external crystal before selecting it as the PLL source. There
 * is deliberately no HSI fallback: HSI is not accurate enough to guarantee
 * USB operation, so continuing with it would create an intermittently
 * failing USB device rather than a usable degraded mode. */
static void HSEEnable(void)
{
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0U)
    {
    }
}

 /*
 * flashInit()
 * ------------
 * Programs FLASH->ACR (Flash Access Control Register). This has to run
 * BEFORE we ever switch SYSCLK up to 84MHz: flash memory itself can't be
 * read fast enough at high core clocks without inserting extra "wait
 * states" into each access. Clock up first and the CPU can under/over-run
 * flash timing and fetch garbage instructions -> hard fault or worse,
 * silent corruption of what "runs".
 */
static void flashInit(void)
{
    FLASH->ACR =
          FLASH_ACR_ICEN
          /* Instruction Cache ENable. Caches recently-fetched instruction
           * lines in a small dedicated flash cache so loops / re-entered
           * functions don't have to re-hit the (now wait-stated, slower)
           * flash array every time. Pure performance, doesn't affect
           * correctness if left off. */
        | FLASH_ACR_DCEN
          /* Data Cache ENable. Same idea as above but for literal
           * pools/const data reads out of flash (e.g. big lookup tables). */
        | FLASH_ACR_PRFTEN
          /* PReFeTCH ENable. Lets the flash interface start fetching the
           * NEXT line while the CPU is still executing out of the current
           * one, hiding some of the extra latency the wait states below
           * add. Works together with ICEN, not a replacement for it. */
        | FLASH_ACR_LATENCY_2WS;
          /* LATENCY = 2 Wait States. Per the STM32F411 datasheet's flash
           * latency table (VCC = 2.7-3.6V column): 0WS is only valid up to
           * ~30MHz, 1WS up to ~64MHz, 2WS up to ~90MHz. We're about to run
           * HCLK at 84MHz, so 2 wait states is the REQUIRED value here -
           * this isn't a tunable performance knob, it's a correctness
           * requirement tied directly to the 84MHz target below. If you
           * ever change the target SYSCLK frequency, this constant has to
           * be revisited against the datasheet table. */
}

/*
 * PLLConfigure()
 * ---------------
 * Programs the PLL's dividers/multiplier/source into RCC->PLLCFGR. This
 * register is only writable while the PLL is OFF and confirmed stopped -
 * on the F4 series PLLCFGR is effectively locked once PLLON is set and the
 * PLL has locked, so we explicitly disable + confirm-stopped first.
 */
static void PLLConfigure(void)
{
    /* Make sure PLL is OFF */

    RCC->CR &= ~RCC_CR_PLLON;
    /* Clear PLLON in RCC->CR (Clock Control Register) - requests PLL
     * power-down. Read-modify-write (&=) rather than a plain store, so we
     * don't disturb the other independent enable bits living in the same
     * register (HSION, HSEON, CSSON, etc). */

    while (RCC->CR & RCC_CR_PLLRDY)
    /* Poll PLLRDY (the PLL lock/ready flag, also in RCC->CR) until hardware
     * clears it. Clearing PLLON is a *request*; it doesn't take effect the
     * same cycle, so we must wait for the hardware-driven ready flag to
     * confirm the PLL has genuinely stopped before touching PLLCFGR. */
    {
    }

    RCC->PLLCFGR =
    /* PLL Configuration Register - one register holding every PLL
     * divider/multiplier/source-select field. Written as a single
     * assignment (not read-modify-write) because we want a fully known,
     * deterministic state rather than layering onto whatever was there
     * before (e.g. leftover bits from boardClockInit_debug_hsi()). */
          (BOARD_PLL_M << RCC_PLLCFGR_PLLM_Pos)
          /* PLLM = 25: input divider applied to the PLL's source clock
           * BEFORE the VCO. Source here is HSE = 25MHz (see PLLSRC below),
           * so VCO input = 25MHz / 25 = 1MHz. (ST's app notes recommend
           * 2MHz VCO-input for best jitter, but 1MHz is within spec and
           * commonly used when you want round PLLN numbers.) */
        | (BOARD_PLL_N << RCC_PLLCFGR_PLLN_Pos)
          /* PLLN = 336: VCO frequency multiplier. VCO output = 1MHz * 336
           * = 336MHz. Must land inside the VCO's valid output range for
           * F411 (100-432MHz) - 336MHz sits comfortably inside that. */
        | (BOARD_PLL_P_BITS << RCC_PLLCFGR_PLLP_Pos)
          /* PLLP is a 2-bit field selecting the MAIN system-clock output
           * divider, encoded as: 00->/2, 01->/4, 10->/6, 11->/8. The value
           * 1 here means binary 01 -> divide by 4. So the PLL's main
           * output (what becomes SYSCLK once selected) = 336MHz / 4 =
           * 84MHz - exactly our target. */
        | RCC_PLLCFGR_PLLSRC_HSE
          /* PLLSRC = HSE: the board's external 25MHz crystal provides the
           * frequency accuracy needed by the USB 48MHz PLLQ output. */
        | (BOARD_PLL_Q << RCC_PLLCFGR_PLLQ_Pos);
          /* PLLQ = 7: a SEPARATE output divider (independent of PLLP) used
           * to feed USB OTG FS / SDIO / RNG. VCO/PLLQ = 336MHz / 7 = 48MHz
           * - the exact frequency USB full-speed requires. Not consumed by
           * anything yet in this codebase, but set correctly up front so
           * enabling USB later doesn't require touching the PLL config. */
}

/*
 * PLLEnable()
 * ------------
 * Powers the already-configured PLL back on and blocks until it reports a
 * stable lock. Must run strictly after PLLConfigure() so PLLM/N/P/Q are
 * already programmed before the PLL starts running.
 */
static void PLLEnable(void)
{
    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    /* Set PLLON in RCC->CR - starts the PLL running with the dividers
     * programmed by PLLConfigure(). Read-modify-write to leave HSION and
     * friends untouched. */

    /* Wait until PLL locks */
    while ((RCC->CR & RCC_CR_PLLRDY) == 0U)
    /* Poll PLLRDY until hardware SETS it this time - this is the lock
     * detector confirming the VCO has settled onto a stable frequency.
     * Switching SYSCLK onto the PLL (next function) before PLLRDY is set
     * would point the whole chip at an unstable/settling clock - never do
     * that switch before this wait returns. */
    {
    }
}

static void sysClockSwitchToPll(void)
{
    /*---------------------------------------------------------
     * Configure Bus Prescalers
     *
     * HCLK  = SYSCLK
     * PCLK1 = HCLK / 2 = 42 MHz
     * PCLK2 = HCLK     = 84 MHz
     *--------------------------------------------------------*/

    RCC->CFGR &= ~(RCC_CFGR_HPRE_Msk |
                   RCC_CFGR_PPRE1_Msk |
                   RCC_CFGR_PPRE2_Msk);
    /* Clear three prescaler fields in RCC->CFGR (Clock Configuration
     * Register) in one shot:
     *   HPRE  - AHB prescaler.  Cleared -> /1, so HCLK = SYSCLK = 84MHz.
     *   PPRE1 - APB1 prescaler. Cleared first so the /2 value below can be
     *           OR'd onto a known-zero field rather than onto whatever
     *           stale bits might already be sitting there.
     *   PPRE2 - APB2 prescaler. Cleared -> /1 and left that way, so
     *           PCLK2 = HCLK = 84MHz directly (APB2 peripherals are rated
     *           high enough to take the full 84MHz without division). */

    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    /* Set APB1 prescaler = /2. APB1 peripherals on the F411 (I2C1, TIM2-5,
     * USART2/3, SPI2/3, etc.) are rated up to 50MHz max. Feeding them the
     * raw 84MHz HCLK would exceed spec, so PCLK1 = 84MHz / 2 = 42MHz
     * instead. This is the register that directly determines the input
     * clock your I2C1 timing-register (CCR/TRISE, or TIMINGR on other
     * families) math is based on - if this ever changes, I2C1 bus timing
     * calculations need to change with it. */

    /*---------------------------------------------------------
     * Switch SYSCLK -> PLL
     *--------------------------------------------------------*/

    RCC->CFGR &= ~RCC_CFGR_SW_Msk;
    /* Clear the SW (System clock switch / source-select request) field.
     * Required before writing a new source selection into it - can't just
     * OR a new source on top of whatever was previously selected. */

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    /* Request PLL as the SYSCLK source (as opposed to HSI or HSE). THIS is
     * the actual clock-source switch - everything before this point (flash
     * latency, PLL config, PLL lock) was preparation; the chip was still
     * running on HSI the whole time until this line executes. */

    while ((RCC->CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL)
    /* Poll SWS (System clock Switch Status) - a separate, READ-ONLY mirror
     * field in the same register - until it confirms the PLL is actually
     * live as SYSCLK. SW is just the request; SWS is hardware's
     * confirmation, and there's a small (few-cycle) delay between writing
     * SW and SWS reflecting it. Never assume the switch is done just
     * because SW was written. */
    {
    }

    SystemCoreClockUpdate(BOARD_SYSCLK_HZ);
    /* Update the firmware-wide "what's the core clock right now" tracking
     * variable to 84MHz. Everything downstream that does timing math off
     * that value (SysTick reload, FreeRTOS tick rate, UART baud-rate
     * divisors, blocking delay loops) depends on this being accurate -
     * the hardware switch above doesn't automatically inform any of that
     * software state. */
}

/*
 * boardClockInit()
 * -------------------
 * Full bring-up sequence for the normal (non-debug) 84MHz clock tree.
 * Order is load-bearing:
 *   1) flashInit()          - wait states set BEFORE running fast
 *   2) HSEEnable()          - external 25MHz crystal started and stable
 *   3) PLLConfigure()       - PLL programmed while OFF
 *   4) PLLEnable()          - PLL turned on and locked
 *   5) sysClockSwitchToPll()- SYSCLK pointed at PLL only once it's locked
 * Doing these out of order can hard-fault the chip (flash) or leave it
 * running on an unstable/unconfigured clock (PLL steps).
 */
void boardClockInit(void)
{
    flashInit();
    /* Step 1: set flash wait states + caches so the CPU can safely fetch
     * instructions once we're actually running at 84MHz. */

    HSEEnable();
    /* Step 2: start the 25MHz external crystal and wait until it is stable. */

    PLLConfigure();
    /* Step 3: program PLLM/PLLN/PLLP/PLLQ/PLLSRC into PLLCFGR while the
     * PLL is confirmed off. */

    PLLEnable();
    /* Step 4: turn the PLL on and block until PLLRDY confirms lock. */

    sysClockSwitchToPll();
    /* Step 5: zero + reprogram the AHB/APB1/APB2 prescalers, then switch
     * SW from HSI to PLL and block until SWS confirms the switch. */

    volatile uint32_t cr      = RCC->CR;
    /* Debug read-back: snapshot of Clock Control Register after the
     * switch - shows HSION/HSIRDY/PLLON/PLLRDY etc, handy to inspect in a
     * debugger's watch/locals window right after bring-up. */
    volatile uint32_t cfgr    = RCC->CFGR;
    /* Debug read-back: snapshot of Clock Configuration Register - SW/SWS
     * should both read "PLL" here, and the prescaler fields should show
     * HPRE=/1, PPRE1=/2, PPRE2=/1 as programmed above. */
    volatile uint32_t pllcfgr = RCC->PLLCFGR;
    /* Debug read-back: snapshot of PLL Configuration Register - confirms
     * PLLM/PLLN/PLLP/PLLQ/PLLSRC actually latched the values written in
     * PLLConfigure(), rather than trusting the write blindly. */

    (void)cr;
    (void)cfgr;
    (void)pllcfgr;
    /* These three locals are never otherwise read by code - they exist
     * purely so a debugger can inspect them. (void) casts silence
     * "set but not used" compiler warnings without disabling the warning
     * class globally. `volatile` above ensures the compiler can't
     * optimize the reads away entirely just because the values are
     * "unused" from its point of view. */

    SystemCoreClockUpdate(BOARD_SYSCLK_HZ);
    /* Redundant second call - sysClockSwitchToPll() already did this.
     * Harmless belt-and-suspenders: guarantees the tracked core-clock
     * value is correct by the time boardClockInit() returns, even if
     * someone edits sysClockSwitchToPll() later and removes its call. */
}

/*----------------------------------------------------------------------
 * DEBUG ONLY: runs everything off the default 16MHz HSI, PLL untouched
 * (left off). Used to rule the PLL/84MHz clock path in or out while
 * chasing the I2C1 SCL issue. Swap the boardInit() call in main.c for
 * boardClockInit_debug_hsi() temporarily, then swap back once done.
 *
 * NOTE: FreeRTOSConfig.h has configCPU_CLOCK_HZ hardcoded to 84000000.
 * Running at 16MHz without updating that will make vTaskDelay() and the
 * scheduler tick roughly 5x slower than they appear (e.g. the I2C scan
 * task's 5s delay becomes ~26s). Expected and harmless for this test,
 * just don't be alarmed if things seem to slow down a lot.
 *--------------------------------------------------------------------*/
void boardClockInit_debug_hsi(void)
{
    flashInit();
    /* Still apply flash caching/prefetch + 2 wait states. Technically 0WS
     * would be legal at 16MHz, but reusing flashInit() as-is just costs a
     * little performance, not correctness - keeps this debug path simple
     * and avoids a second flash-latency code path to maintain. */

    /* Make sure PLL is off - HSI should already be the active source at
     * reset, but be explicit in case boardClockInit() ran first. */
    RCC->CR &= ~RCC_CR_PLLON;
    /* Request PLL power-down, in case boardClockInit() already started
     * and locked it earlier (e.g. you're switching clock paths at runtime
     * for A/B testing rather than only at boot). */
    while (RCC->CR & RCC_CR_PLLRDY)
    /* Wait for PLLRDY to clear, confirming the PLL has genuinely stopped,
     * before relying on HSI alone as SYSCLK below. */
    {
    }

    RCC->CFGR &= ~(RCC_CFGR_HPRE_Msk |
                   RCC_CFGR_PPRE1_Msk |
                   RCC_CFGR_PPRE2_Msk);
    /* Clear AHB/APB1/APB2 prescaler fields.
     *   HPRE  cleared -> /1, so HCLK = SYSCLK = HSI = 16MHz once switched.
     *   PPRE1 cleared first so /2 can be set explicitly below.
     *   PPRE2 cleared -> /1, left there, so PCLK2 = HCLK = 16MHz. */

    /* Keep PCLK1 = SYSCLK/2, matching the fixed assumption baked into
     * SystemCoreClockUpdate() below (pclk1 = clock/2). SYSCLK=16MHz here
     * -> PCLK1=8MHz, PCLK2=16MHz. Both comfortably valid for I2C1. */
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    /* APB1 (where I2C1 lives) = 16MHz / 2 = 8MHz. Deliberately keeps the
     * same /2 relationship used in the 84MHz path, so I2C1's timing
     * register math only has to scale by the PCLK1 value itself and
     * doesn't need a second formula/code path for this debug clock tree. */

    RCC->CFGR &= ~RCC_CFGR_SW_Msk;
    /* Clear SW field before writing a new source selection into it. */
    RCC->CFGR |= RCC_CFGR_SW_HSI;
    /* Explicitly request HSI as SYSCLK source. Belt-and-suspenders: HSI is
     * the reset default anyway, but if boardClockInit() ran first and
     * switched SW to PLL, this is what actually switches back to HSI. */

    while ((RCC->CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_HSI)
    /* Wait for the read-only SWS status field to confirm HSI is actually
     * live as SYSCLK (same SW-vs-SWS request/confirm pattern as the PLL
     * path above). */
    {
    }

    SystemCoreClockUpdate(16000000U);
    /* Tell the rest of the firmware the core clock is now 16MHz. Critical
     * for anything doing timing math (baud rates, I2C timing registers,
     * software delay loops) - see the big warning above re: FreeRTOS tick
     * rate running ~5x slow while configCPU_CLOCK_HZ still assumes 84MHz. */
}

/*
 * boardInit()
 * ------------
 * Top-level hardware bring-up entry point, meant to be called once from
 * main(). Currently clocks are the only board-wide thing that needs init,
 * so this is a thin wrapper - kept as a separate function (rather than
 * having main() call boardClockInit() directly) so other board-level
 * setup (GPIO reset states, power/voltage-scaling config, etc.) can be
 * added here later without touching every existing call site.
 */
void boardInit(void)
{
    boardClockInit();
    /* Bring up flash wait states + PLL + 84MHz SYSCLK / 42MHz PCLK1 /
     * 84MHz PCLK2, i.e. the normal (non-debug) clock tree. */
}
