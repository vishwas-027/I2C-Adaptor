#ifndef BOARD_H   /* Include guard start - prevents this header's contents being pasted twice if board.h is #included from more than one place in the same translation unit. */
#define BOARD_H   /* Defines the guard macro; on any subsequent #include of this file, BOARD_H is already defined so the whole body is skipped. */

/* Initialize all board-wide hardware. Currently this configures the clocks. */
void boardInit(void);
/* ^ Public entry point meant to be called once from main() (or your startup
 *   code) before anything else runs. Right now it's a thin wrapper around
 *   boardClockInit(), but it's kept as its own function/name so that later
 *   board bring-up (GPIO default states, power regulator config, etc.) can
 *   be added inside boardInit() without changing every call site that
 *   currently calls it expecting "clocks are set up after this returns". */

/* Configure the 25 MHz HSE + PLL for 84 MHz SYSCLK, 42 MHz PCLK1,
 * 84 MHz PCLK2, and the 48 MHz PLLQ output required by USB OTG FS. */
void boardClockInit(void);
/* ^ The "real" clock-tree bring-up: sets flash wait states, configures and
 *   locks the PLL (fed from the internal 16MHz HSI oscillator), then
 *   switches SYSCLK over to the PLL output and sets the APB1/APB2 bus
 *   prescalers. See board.c for the register-by-register breakdown.
 *   - SYSCLK = 84MHz (from PLL: HSE/PLLM * PLLN / PLLP = 25/25*336/4)
 *   - HCLK   = 84MHz (AHB prescaler = /1)
 *   - PCLK1  = 42MHz (APB1 prescaler = /2)  <- feeds I2C1, TIM2-5, USART2/3
 *   - PCLK2  = 84MHz (APB2 prescaler = /1)  <- feeds TIM1, USART1/6, SPI1
 *   - PLLQ   = 48MHz (USB OTG FS clock) */

/* DEBUG ONLY - see board.c for details */
void boardClockInit_debug_hsi(void);
/* ^ Alternate bring-up path that deliberately leaves the PLL OFF and runs
 *   the whole chip off the raw 16MHz HSI oscillator (PCLK1=8MHz,
 *   PCLK2=16MHz). Intended to be swapped in for boardClockInit() in
 *   main.c temporarily, to rule the PLL/84MHz clock path in or out while
 *   debugging hardware issues (e.g. the I2C1 SCL problem). Remember:
 *   FreeRTOSConfig.h's configCPU_CLOCK_HZ is hardcoded to 84000000, so the
 *   RTOS tick/vTaskDelay() will run ~5x slower than normal while this is
 *   active - expected, not a bug. */

#endif /* BOARD_H - include guard end. */
