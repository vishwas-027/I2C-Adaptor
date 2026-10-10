/*
 * i2c.c
 *
 * Bare-metal I2C1 master driver for STM32F411CEU6 (BlackPill).
 * See i2c.h for API documentation and pin mapping.
 */

#include "i2c.h"
#include "logger.h"
#include "../board/system.h"  /* Supplies the live APB1 clock for timing. */

#include <stddef.h>
#include <stdio.h>

/* NOTE: PCLK1 is now read from the live SystemClock struct set up by
 * boardClockInit(), instead of being hardcoded. This was the actual bug:
 * board.c configures the PLL for 84MHz SYSCLK / 42MHz PCLK1, but this file
 * previously assumed 16MHz HSI with no PLL, which corrupted every I2C
 * timing register (CR2.FREQ, CCR, TRISE). */

/* ------------------------------------------------------------------------ */
/* Private helpers                                                          */
/* ------------------------------------------------------------------------ */

static I2C_Speed_t s_configured_speed = I2C_SPEED_STANDARD_100K;
static uint32_t    s_configured_pclk1_hz = 0;
static uint8_t s_slave_address;
static I2C1_SlaveReceiveCallback s_slave_callback;
static I2C1_SlaveEventCallback s_slave_event_callback;
static volatile uint8_t s_slave_frame[I2C1_SLAVE_RX_MAX_LENGTH];
static volatile uint16_t s_slave_frame_length;
static volatile bool s_slave_receiving;
static volatile bool s_slave_transmitting;
static volatile uint16_t s_slave_tx_count;
static volatile uint32_t s_slave_tx_index;
static uint16_t s_slave_max_tx_bytes = I2C1_SLAVE_TX_MAX_LENGTH;
static uint8_t s_slave_response[I2C1_SLAVE_TX_MAX_LENGTH];
static volatile uint16_t s_slave_response_length;
static uint16_t s_slave_max_rx_bytes = I2C1_SLAVE_RX_MAX_LENGTH;
/* Tracks whether the blocking master driver currently owns the bus, so the
 * event ISR can detect "this is a spurious SB/TXE from my own polling
 * master transfer" without reading SR2. Reading SR2 to check SR2_MSL was
 * previously done unconditionally at ISR entry - but reading SR2 right
 * after SR1 is also the hardware sequence that clears ADDR, so a genuine
 * slave-mode ADDR interrupt had ADDR cleared as a side effect of this check
 * before the code even branched into the code that was supposed to be
 * handling it. Tracking master ownership in software avoids touching SR2
 * at all except inside the ADDR branch, exactly when it's needed. */
static volatile bool s_master_transaction_active;

/* Slave-mode event counters for I2C1_EV_IRQHandler and I2C1_ER_IRQHandler,
 * reset by I2C1_ResetSlaveEventCounters() right before each wait for an
 * incoming response. Same diagnostic approach as the endpoint's master-mode
 * counters: on a receive timeout, these show whether this board's slave
 * hardware saw ANY activity at all during the wait, and if so, which event
 * last arrived before things went quiet.
 *   ev_isr_entries == 0    the event ISR was never entered in slave mode
 *                          during the whole wait - nothing reached this
 *                          board's I2C1 as far as the peripheral is
 *                          concerned (compare against the endpoint's own
 *                          sb=1 - it believes it started addressing us).
 *   addr_events == 0       entered, but ADDR was never the bit seen -
 *                          check last_sr1_seen.
 *   addr_events > 0,
 *   stopf_events == 0      address matched, but the transaction never
 *                          reached STOPF - stalled mid-frame.
 *   er_isr_entries > 0     the error ISR ran - check last_sr1_seen there
 *                          for which error bit(s) were set (only
 *                          meaningful now that ITERREN is kept enabled). */
static volatile uint32_t s_slave_ev_isr_entries;
static volatile uint32_t s_slave_addr_events;
static volatile uint32_t s_slave_rxne_events;
static volatile uint32_t s_slave_stopf_events;
static volatile uint32_t s_slave_unmatched_events;
static volatile uint32_t s_slave_last_sr1_seen;
static volatile uint32_t s_slave_er_isr_entries;
static volatile uint32_t s_slave_er_last_sr1_seen;
/* How many times the error ISR fired *during* a master transaction (BERR or
 * ARLO seen with s_master_transaction_active true) - see the comment in
 * I2C1_ER_IRQHandler(). Should normally be 0; any nonzero count confirms a
 * real bus-error/arbitration-loss event happened mid-scan-or-transfer, which
 * is exactly the condition that used to livelock the CPU before this flag
 * started being cleared unconditionally. */
static volatile uint32_t s_master_er_isr_entries;
static volatile uint32_t s_master_error_flags;

static void I2C1_ConfigureRegisters(uint32_t pclk1_hz, I2C_Speed_t speed);
static const char *I2C1_Sr1Flags(uint32_t sr1, char *buffer, size_t buffer_size);
static void I2C1_LogMasterFault(const char *context, uint32_t sr1, uint32_t sr2);
static I2C_ExtendedStatus_t I2C1_MasterExtendedFault(void);

static const char *I2C1_Sr1Flags(uint32_t sr1, char *buffer, size_t buffer_size)
{
    static const struct {
        uint32_t mask;
        const char *name;
    } flags[] = {
        { I2C_SR1_SB, "SB" },
        { I2C_SR1_ADDR, "ADDR" },
        { I2C_SR1_BTF, "BTF" },
        { I2C_SR1_STOPF, "STOPF" },
        { I2C_SR1_RXNE, "RXNE" },
        { I2C_SR1_TXE, "TXE" },
        { I2C_SR1_BERR, "BERR" },
        { I2C_SR1_ARLO, "ARLO" },
        { I2C_SR1_AF, "AF" },
        { I2C_SR1_OVR, "OVR" },
        { I2C_SR1_PECERR, "PECERR" },
        { I2C_SR1_TIMEOUT, "TIMEOUT" },
        { I2C_SR1_SMBALERT, "SMBALERT" }
    };
    size_t used = 0U;
    bool first = true;

    if ((buffer == NULL) || (buffer_size == 0U)) {
        return "";
    }

    buffer[0] = '\0';
    for (size_t index = 0U; index < (sizeof(flags) / sizeof(flags[0])); index++) {
        int written;

        if ((sr1 & flags[index].mask) == 0U) {
            continue;
        }
        written = snprintf(buffer + used, buffer_size - used, "%s%s",
                           first ? "" : "|", flags[index].name);
        if (written < 0) {
            break;
        }
        if ((size_t)written >= (buffer_size - used)) {
            used = buffer_size - 1U;
            break;
        }
        used += (size_t)written;
        first = false;
    }

    if (first) {
        (void)snprintf(buffer, buffer_size, "none");
    }
    return buffer;
}

static void I2C1_LogMasterFault(const char *context, uint32_t sr1, uint32_t sr2)
{
    char flags[64];

    LOG_ERROR("%s SR1=0x%04lX (%s) SR2=0x%04lX",
              context,
              (unsigned long)sr1,
              I2C1_Sr1Flags(sr1, flags, sizeof(flags)),
              (unsigned long)sr2);
}

static void I2C1_EnableEventInterrupt(uint8_t irq_number)
{
    volatile uint32_t *iser = (volatile uint32_t *)(NVIC_BASE + 0x000UL);
    volatile uint8_t *ipr = (volatile uint8_t *)(NVIC_BASE + 0x300UL);

    ipr[irq_number] = 0x50U; /* FreeRTOS syscall-safe priority (logical 5). */
    iser[irq_number / 32U] = (1UL << (irq_number % 32U));
}

static void I2C1_GPIO_Init(void)
{
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);

    /* PB8 = SCL, PB9 = SDA -> Alternate Function mode (MODER = 10)
     * Switched from PB6/PB7 while isolating a persistent SCL fault -
     * PB8/PB9 is I2C1's other AF4 pin pair on the F411 (datasheet
     * DocID026289, table 8). */
    MODIFY_REG(GPIOB->MODER,
               (0x3U << (8 * 2)) | (0x3U << (9 * 2)),
               (0x2U << (8 * 2)) | (0x2U << (9 * 2)));

    /* Open-drain output type - required for I2C bus lines */
    SET_BIT(GPIOB->OTYPER, (1U << 8) | (1U << 9));

    /* High speed */
    MODIFY_REG(GPIOB->OSPEEDR,
               (0x3U << (8 * 2)) | (0x3U << (9 * 2)),
               (0x3U << (8 * 2)) | (0x3U << (9 * 2)));

    /* No internal pull - this bus now runs a real cable to a second board,
     * exactly the case this comment used to warn about. Relying on the
     * STM32's weak internal pull-up (tens of kOhm) here is the leading
     * suspect for a bus that hangs stretched low specifically during the
     * endpoint's response TX: it's the only device on this bus that CAN
     * supply a pull-up at all - the F103 endpoint's AF open-drain pins have
     * no internal pull-up option, so it depends entirely on this bus having
     * real external resistors.
     *
     * REQUIRES external ~4.7k pull-ups to 3.3V on SCL and SDA, shared by
     * both boards, matching the values already worked out during earlier
     * bring-up. Do not flash this without confirming those resistors are
     * physically on the bus - internal pull-up alone is safer than no
     * pull-up at all, and removing it here without a real external
     * resistor in place will make the bus worse, not better. */
    MODIFY_REG(GPIOB->PUPDR,
               (0x3U << (8 * 2)) | (0x3U << (9 * 2)),
               (0x0U << (8 * 2)) | (0x0U << (9 * 2)));

    /* AF4 = I2C1. PB8 and PB9 fall in AFR[1] (AFRH) since pin >= 8. */
    MODIFY_REG(GPIOB->AFR[1],
               (0xFU << ((8 - 8) * 4)) | (0xFU << ((9 - 8) * 4)),
               (0x4U << ((8 - 8) * 4)) | (0x4U << ((9 - 8) * 4)));
}

/* Generates a START condition.
 * wait_for_bus_free: true for a fresh transaction (BUSY must be 0),
 *                     false for a repeated START (we already own the bus). */
static I2C_Status_t I2C1_GenerateStart(bool wait_for_bus_free)
{
    uint32_t timeout;

    if (wait_for_bus_free) {
        timeout = I2C1_TIMEOUT_DEFAULT;
        while (READ_BIT(I2C1->SR2, I2C_SR2_BUSY) && --timeout) { }
        if (timeout == 0) return I2C_TIMEOUT;
    }

    /* Set before START so the very first SB interrupt (which can arrive
     * before this function returns) already sees master mode as active. */
    s_master_transaction_active = true;
    SET_BIT(I2C1->CR1, I2C_CR1_START);

    timeout = I2C1_TIMEOUT_DEFAULT;
    while (!READ_BIT(I2C1->SR1, I2C_SR1_SB) && --timeout) { }
    if (timeout == 0) {
        s_master_transaction_active = false;
        return I2C_TIMEOUT;
    }

    return I2C_OK;
}

/* Request STOP without resetting I2C1. A receive transaction must read the
 * final bytes from DR before the peripheral can safely be reconfigured. */
static void I2C1_RequestStop(void)
{
    SET_BIT(I2C1->CR1, I2C_CR1_STOP);
}

static void I2C1_WaitForStopAndRecover(void)
{
    uint32_t timeout = I2C1_TIMEOUT_DEFAULT;

    while (READ_BIT(I2C1->CR1, I2C_CR1_STOP) && --timeout) { }

    /* STM32F411 errata 2.9.3 recovery, applied immediately after our own
     * STOP rather than deferred to the next outbound master START.
     *
     * Deferring it left this peripheral going straight from "just finished
     * being bus master" into slave-listen mode with no reset in between -
     * and its internal state machine doesn't reliably let go of "I'm still
     * mid-transfer" across that particular transition. Confirmed by
     * instrumentation: the very next time a peer addressed us, this
     * peripheral's error ISR fired a genuine BERR (bus error - "Start/Stop
     * seen while I thought I was mid-byte"), while the peer's own BUSY read
     * independently showed the bus was truly idle at that instant. That
     * rules out an actual bus problem; it was this peripheral's own leftover
     * state. I2C1_ConfigureRegisters() re-arms OAR1/ACK/PE/interrupts as
     * part of the same reset, so this is a full replacement for the manual
     * re-arm this function used to do, not an addition to it. The reset
     * itself is a handful of register writes, so the address-miss window
     * this deferral used to avoid should now be negligible. */
    I2C1_ConfigureRegisters(s_configured_pclk1_hz, s_configured_speed);

    /* Bus is back in slave-listen mode - clear last, after
     * I2C1_ConfigureRegisters() has already re-armed OAR1/interrupts, so
     * there's no window where a genuine incoming ADDR could be missed as
     * still "ours". */
    s_master_transaction_active = false;
}

static I2C_ExtendedStatus_t I2C1_ReceiveCoreExt(uint8_t dev_addr,
                                                uint8_t *data,
                                                uint16_t len,
                                                bool repeated_start,
                                                uint16_t *num_read)
{
    I2C_Status_t status;
    I2C_ExtendedStatus_t extended;
    uint32_t timeout;
    uint16_t i = 0;

    if ((data == NULL) || (len == 0U) || (num_read == NULL)) {
        return I2C_EXT_STATUS_BUS_ERROR;
    }
    *num_read = 0U;
    s_master_error_flags = 0U;

    status = I2C1_GenerateStart(!repeated_start);
    if (status != I2C_OK) {
        extended = I2C1_MasterExtendedFault();
        return (extended != I2C_EXT_STATUS_OK)
                   ? extended : I2C_EXT_STATUS_BUS_LOCKED;
    }

    /* Single-byte reads need NACK armed before the address is even sent.
     * Two-byte reads also require POS before ADDR is cleared. */
    if (len == 1) {
        CLEAR_BIT(I2C1->CR1, I2C_CR1_ACK);
    } else if (len == 2) {
        SET_BIT(I2C1->CR1, I2C_CR1_POS);
        CLEAR_BIT(I2C1->CR1, I2C_CR1_ACK);
    } else {
        CLEAR_BIT(I2C1->CR1, I2C_CR1_POS);
        SET_BIT(I2C1->CR1, I2C_CR1_ACK);
    }

    status = I2C1_SendAddress(dev_addr, 1);
    if (status != I2C_OK) {
        /* I2C1_SendAddress() already issues STOP on its own failure paths.
         * This function may have cleared ACK/set POS above (for the
         * len==1/len==2 cases) before the address phase was even attempted -
         * restore both before returning, or a NACK'd target here would leave
         * ACK permanently cleared, silently NACKing every future address
         * match once this peripheral goes back to listening as a slave. */
        CLEAR_BIT(I2C1->CR1, I2C_CR1_POS);
        SET_BIT(I2C1->CR1, I2C_CR1_ACK);
        extended = I2C1_MasterExtendedFault();
        if (extended != I2C_EXT_STATUS_OK) {
            return extended;
        }
        return (status == I2C_NACK) ? I2C_EXT_STATUS_SLA_NACK
                                    : I2C_EXT_STATUS_BUS_LOCKED;
    }

    if (len == 1) {
        I2C1_RequestStop();
        timeout = I2C1_TIMEOUT_DEFAULT;
        while (!READ_BIT(I2C1->SR1, I2C_SR1_RXNE) && --timeout) {
            extended = I2C1_MasterExtendedFault();
            if (extended != I2C_EXT_STATUS_OK) goto fault;
        }
        extended = I2C1_MasterExtendedFault();
        if (extended != I2C_EXT_STATUS_OK) goto fault;
        if (timeout == 0U) goto locked;
        data[0] = (uint8_t)I2C1->DR;
        *num_read = 1U;
        I2C1_WaitForStopAndRecover();
    } else if (len == 2) {
        /* For two bytes, wait for both shift registers to fill before
         * generating STOP, then read both bytes back-to-back. */
        timeout = I2C1_TIMEOUT_DEFAULT;
        while (!READ_BIT(I2C1->SR1, I2C_SR1_BTF) && --timeout) {
            extended = I2C1_MasterExtendedFault();
            if (extended != I2C_EXT_STATUS_OK) goto fault;
        }
        extended = I2C1_MasterExtendedFault();
        if (extended != I2C_EXT_STATUS_OK) goto fault;
        if (timeout == 0U) goto locked;

        I2C1_RequestStop();
        data[0] = (uint8_t)I2C1->DR;
        data[1] = (uint8_t)I2C1->DR;
        *num_read = 2U;
        I2C1_WaitForStopAndRecover();
    } else {
        /* RM0383: receive ordinary bytes one at a time. Leave exactly
         * three pending, which need the BTF/ACK/STOP sequence below. */
        while ((uint16_t)(len - i) > 3U) {
            timeout = I2C1_TIMEOUT_DEFAULT;
            while (!READ_BIT(I2C1->SR1, I2C_SR1_RXNE) && --timeout) {
                extended = I2C1_MasterExtendedFault();
                if (extended != I2C_EXT_STATUS_OK) goto fault;
            }
            extended = I2C1_MasterExtendedFault();
            if (extended != I2C_EXT_STATUS_OK) goto fault;
            if (timeout == 0U) goto locked;
            data[i++] = (uint8_t)I2C1->DR;
            *num_read = i;
        }

        timeout = I2C1_TIMEOUT_DEFAULT;
        while (!READ_BIT(I2C1->SR1, I2C_SR1_BTF) && --timeout) {
            extended = I2C1_MasterExtendedFault();
            if (extended != I2C_EXT_STATUS_OK) goto fault;
        }
        extended = I2C1_MasterExtendedFault();
        if (extended != I2C_EXT_STATUS_OK) goto fault;
        if (timeout == 0U) goto locked;

        CLEAR_BIT(I2C1->CR1, I2C_CR1_ACK);
        data[i++] = (uint8_t)I2C1->DR;
        *num_read = i;

        timeout = I2C1_TIMEOUT_DEFAULT;
        while (!READ_BIT(I2C1->SR1, I2C_SR1_BTF) && --timeout) {
            extended = I2C1_MasterExtendedFault();
            if (extended != I2C_EXT_STATUS_OK) goto fault;
        }
        extended = I2C1_MasterExtendedFault();
        if (extended != I2C_EXT_STATUS_OK) goto fault;
        if (timeout == 0U) goto locked;

        I2C1_RequestStop();
        data[i++] = (uint8_t)I2C1->DR;
        *num_read = i;
        data[i++] = (uint8_t)I2C1->DR;
        *num_read = i;
        I2C1_WaitForStopAndRecover();
    }

    CLEAR_BIT(I2C1->CR1, I2C_CR1_POS);
    SET_BIT(I2C1->CR1, I2C_CR1_ACK); /* restore ACK default for next transaction */
    return I2C_EXT_STATUS_OK;

fault:
    I2C1_Stop();
    CLEAR_BIT(I2C1->CR1, I2C_CR1_POS);
    SET_BIT(I2C1->CR1, I2C_CR1_ACK);
    return extended;

locked:
    I2C1_Stop();
    CLEAR_BIT(I2C1->CR1, I2C_CR1_POS);
    SET_BIT(I2C1->CR1, I2C_CR1_ACK);
    return I2C_EXT_STATUS_BUS_LOCKED;
}

static I2C_Status_t I2C1_ReceiveCore(uint8_t dev_addr,
                                     uint8_t *data,
                                     uint16_t len,
                                     bool repeated_start)
{
    uint16_t num_read;

    switch (I2C1_ReceiveCoreExt(dev_addr, data, len, repeated_start,
                                &num_read)) {
    case I2C_EXT_STATUS_OK:         return I2C_OK;
    case I2C_EXT_STATUS_SLA_NACK:   return I2C_NACK;
    case I2C_EXT_STATUS_ARB_LOST:   return I2C_ARBITRATION_LOST;
    case I2C_EXT_STATUS_BUS_LOCKED: return I2C_TIMEOUT;
    case I2C_EXT_STATUS_BUS_ERROR:
    case I2C_EXT_STATUS_SLA_ACK:
    case I2C_EXT_STATUS_DATA_NACK:
    case I2C_EXT_STATUS_LAST_DATA_ACK:
    default:                        return I2C_BUS_ERROR;
    }
}

/* Full register (re)configuration, including SWRST. Split out of
 * I2C1_Init() so it can also be run after every transaction - this is
 * ST's own documented workaround for STM32F411 errata 2.9.3 ("Start
 * cannot be generated after a misplaced Stop"): the peripheral can wedge
 * itself after a Start/Stop sequence such that every subsequent START
 * silently sets SB but never actually clocks anything. The fix is to
 * force a real SWRST between transactions, not just wait on flags. */
static void I2C1_ConfigureRegisters(uint32_t pclk1_hz, I2C_Speed_t speed)
{
    /* Software reset, per RM0383 recommendation before configuring */
    SET_BIT(I2C1->CR1, I2C_CR1_SWRST);
    __asm__ volatile("nop");
    CLEAR_BIT(I2C1->CR1, I2C_CR1_SWRST);

    /* CR2.FREQ = APB1 clock in MHz (bits [5:0]) */
    MODIFY_REG(I2C1->CR2, 0x3FU, (pclk1_hz / 1000000UL) & 0x3FU);

    if (speed == I2C_SPEED_FAST_400K) {
        uint32_t denom = 3UL * (uint32_t)speed;
        uint32_t ccr = (pclk1_hz + denom - 1UL) / denom;  /* ceiling division */
        if (ccr < 1UL) ccr = 1UL;

        MODIFY_REG(I2C1->CCR, 0xFFFU, ccr & 0xFFFU);
        SET_BIT(I2C1->CCR, I2C_CCR_FS);      /* Fast mode */
        CLEAR_BIT(I2C1->CCR, I2C_CCR_DUTY);  /* Tlow/Thigh = 2 */

        I2C1->TRISE = (uint32_t)((((pclk1_hz / 1000000UL) * 300UL) / 1000UL) + 1UL) & 0x3FU;
    } else {
        uint32_t denom = 2UL * (uint32_t)speed;
        uint32_t ccr = (pclk1_hz + denom - 1UL) / denom;  /* ceiling division */
        if (ccr < 4UL) ccr = 4UL;

        MODIFY_REG(I2C1->CCR, 0xFFFU, ccr & 0xFFFU);
        CLEAR_BIT(I2C1->CCR, I2C_CCR_FS);    /* Standard mode */

        I2C1->TRISE = (uint32_t)((pclk1_hz / 1000000UL) + 1UL) & 0x3FU;
    }

    /* PE must be enabled BEFORE writing ACK - this was the actual bug this
     * whole time. Writing ACK while PE=0 doesn't reliably latch on this
     * peripheral, so every SET_BIT(CR1, ACK) here was a silent no-op: PE
     * got set to 1 afterward with ACK left at 0 from the SWRST above.
     * Confirmed by direct register read at the moment of listening:
     * CR1=0x0001 (PE=1, ACK=0) every single time, regardless of pull-ups,
     * clock configuration, or reset timing - none of which could matter,
     * because ACK was never actually being set in the first place. OAR1
     * and the interrupt enables aren't order-sensitive the same way, but
     * are moved below PE too for consistency. */
    SET_BIT(I2C1->CR1, I2C_CR1_PE);
    SET_BIT(I2C1->CR1, I2C_CR1_ACK);
    if (s_slave_callback != NULL) {
        /* Bit 14 must be set in 7-bit address mode on STM32F4. */
        I2C1->OAR1 = (1U << 14) | ((uint32_t)s_slave_address << 1U);
        /* SWRST above wiped CR2, including ITERREN - re-enable it here too,
         * not just ITEVTEN/ITBUFEN. This function runs before every master
         * transaction once recovery is pending, so leaving ITERREN off here
         * meant it silently stayed off for the rest of the program after the
         * very first transmit: I2C1_ER_IRQHandler never ran again, so a real
         * slave-side bus error (BERR/ARLO/AF/...) would never get cleared. */
        SET_BIT(I2C1->CR2, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
    }
}

void I2C1_Init(I2C_Speed_t speed)
{
    uint32_t pclk1_hz = SystemClock.pclk1;

    I2C1_GPIO_Init();

    /* Enable I2C1 peripheral clock */
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);

    s_configured_speed = speed;
    s_configured_pclk1_hz = pclk1_hz;

    I2C1_ConfigureRegisters(pclk1_hz, speed);

    LOG_INFO("I2C1: init done (%s), CCR/TRISE configured for PCLK1=%luMHz",
              (speed == I2C_SPEED_FAST_400K) ? "400kHz Fast" : "100kHz Standard",
              (unsigned long)(pclk1_hz / 1000000UL));

    /* This driver intentionally has no FreeRTOS dependency. Its callers must
     * serialize full transactions; the CLI does so with its own mutex. */
}

I2C_Status_t I2C1_SetBitrate(uint32_t bitrate_hz)
{
    I2C_Speed_t speed;

    if (bitrate_hz == (uint32_t)I2C_SPEED_STANDARD_100K) {
        speed = I2C_SPEED_STANDARD_100K;
    } else if (bitrate_hz == (uint32_t)I2C_SPEED_FAST_400K) {
        speed = I2C_SPEED_FAST_400K;
    } else {
        return I2C_ERROR;
    }

    if (speed == s_configured_speed) {
        return I2C_OK;
    }
    if (READ_BIT(I2C1->SR2, I2C_SR2_BUSY) != 0U) {
        return I2C_BUSY;
    }

    I2C1_ConfigureRegisters(s_configured_pclk1_hz, speed);
    s_configured_speed = speed;
    LOG_INFO("I2C1: bitrate changed to %lu Hz",
             (unsigned long)bitrate_hz);
    return I2C_OK;
}

uint32_t I2C1_GetBitrate(void)
{
    return (uint32_t)s_configured_speed;
}

bool I2C1_EnableSlaveReceive(uint8_t own_address,
                             I2C1_SlaveReceiveCallback callback)
{
    return I2C1_ConfigureSlave(own_address,
                               I2C1_SLAVE_TX_MAX_LENGTH,
                               I2C1_SLAVE_RX_MAX_LENGTH,
                               callback) == I2C_OK;
}

I2C_Status_t I2C1_ConfigureSlave(uint8_t own_address,
                                 uint16_t max_tx_bytes,
                                 uint16_t max_rx_bytes,
                                 I2C1_SlaveReceiveCallback callback)
{
    if ((own_address > 0x7FU) || (callback == NULL) ||
        (max_tx_bytes == 0U) ||
        (max_tx_bytes > I2C1_SLAVE_TX_MAX_LENGTH) ||
        (max_rx_bytes == 0U) ||
        (max_rx_bytes > I2C1_SLAVE_RX_MAX_LENGTH)) {
        return I2C_ERROR;
    }
    if (READ_BIT(I2C1->SR2, I2C_SR2_BUSY) != 0U) {
        return I2C_BUSY;
    }

    s_slave_address = own_address;
    s_slave_callback = callback;
    s_slave_max_tx_bytes = max_tx_bytes;
    s_slave_max_rx_bytes = max_rx_bytes;
    s_slave_frame_length = 0U;
    s_slave_receiving = false;
    s_slave_transmitting = false;
    s_slave_tx_count = 0U;
    s_slave_tx_index = 0U;

    CLEAR_BIT(I2C1->CR1, I2C_CR1_PE);
    I2C1->OAR1 = (1U << 14) | ((uint32_t)own_address << 1U);
    /* PE must be set before ACK for the ACK write to actually latch (see
     * I2C1_ConfigureRegisters()) - this call already relies on I2C1_Init()
     * having run first in the current call order, but set it explicitly
     * here too rather than depend on that ordering silently. */
    SET_BIT(I2C1->CR1, I2C_CR1_PE);
    SET_BIT(I2C1->CR1, I2C_CR1_ACK);
    SET_BIT(I2C1->CR2, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
    LOG_INFO("I2C1: slave enabled address=0x%02X tx_max=%u rx_max=%u",
             own_address, (unsigned int)max_tx_bytes,
             (unsigned int)max_rx_bytes);
    I2C1_EnableEventInterrupt(31U); /* I2C1_EV_IRQn */
    I2C1_EnableEventInterrupt(32U); /* I2C1_ER_IRQn */
    return I2C_OK;
}

I2C_Status_t I2C1_SetSlaveResponse(const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U) ||
        (length > s_slave_max_tx_bytes) || s_slave_transmitting) {
        return I2C_ERROR;
    }
    for (uint16_t index = 0U; index < length; index++) {
        s_slave_response[index] = data[index];
    }
    s_slave_response_length = length;
    return I2C_OK;
}

void I2C1_SetSlaveEventCallback(I2C1_SlaveEventCallback callback)
{
    s_slave_event_callback = callback;
}

/* Call right before blocking to wait for an incoming response, so the
 * counters below reflect exactly that wait window. */
void I2C1_ResetSlaveEventCounters(void)
{
    s_slave_ev_isr_entries = 0U;
    s_slave_addr_events = 0U;
    s_slave_rxne_events = 0U;
    s_slave_stopf_events = 0U;
    s_slave_unmatched_events = 0U;
    s_slave_last_sr1_seen = 0U;
    s_slave_er_isr_entries = 0U;
    s_slave_er_last_sr1_seen = 0U;
    s_master_er_isr_entries = 0U;
}

uint32_t I2C1_GetSlaveEvIsrEntries(void)   { return s_slave_ev_isr_entries; }
uint32_t I2C1_GetSlaveAddrEvents(void)     { return s_slave_addr_events; }
uint32_t I2C1_GetSlaveRxneEvents(void)     { return s_slave_rxne_events; }
uint32_t I2C1_GetSlaveStopfEvents(void)    { return s_slave_stopf_events; }
uint32_t I2C1_GetSlaveUnmatchedEvents(void){ return s_slave_unmatched_events; }
uint32_t I2C1_GetSlaveLastSr1Seen(void)    { return s_slave_last_sr1_seen; }
uint32_t I2C1_GetSlaveErIsrEntries(void)   { return s_slave_er_isr_entries; }
uint32_t I2C1_GetSlaveErLastSr1Seen(void)  { return s_slave_er_last_sr1_seen; }
uint32_t I2C1_GetMasterErIsrEntries(void)  { return s_master_er_isr_entries; }

/* Direct register snapshot, not inference from software state - call right
 * when listening begins, to settle definitively whether PE/ACK/OAR1 are
 * actually what the driver believes they are at that exact moment. */
void I2C1_GetSlaveModeRegisters(uint32_t *cr1, uint32_t *oar1, uint32_t *sr2)
{
    if (cr1 != NULL)  { *cr1 = I2C1->CR1; }
    if (oar1 != NULL) { *oar1 = I2C1->OAR1; }
    if (sr2 != NULL)  { *sr2 = I2C1->SR2; }
}

void I2C1_EV_IRQHandler(void)
{
    uint32_t sr1 = I2C1->SR1;

    /* Master transactions remain owned by the blocking master driver. Uses
     * the software flag, not an SR2 read, so this check never has the side
     * effect of clearing a pending ADDR before the branch below gets to it -
     * see the flag's declaration comment. */
    if (s_master_transaction_active) {
        /* TXE/SB events remain asserted throughout a polling master transfer.
         * Leaving event IRQs enabled would continuously re-enter this handler
         * and starve FreeRTOS. I2C1_WaitForStopAndRecover() restores them. */
        CLEAR_BIT(I2C1->CR2, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);
        return;
    }

    if (READ_BIT(sr1, I2C_SR1_ADDR)) {
        uint32_t sr2;
        /* Clear ADDR by the required SR1/SR2 read sequence: SR1 was already
         * read above: this SR2 read is the only place in the whole ISR that
         * touches SR2, and it happens exactly when ADDR is being handled. */
        sr2 = I2C1->SR2;
        s_slave_ev_isr_entries++;
        s_slave_last_sr1_seen = sr1;
        s_slave_addr_events++;
        s_slave_frame_length = 0U;
        s_slave_tx_count = 0U;
        s_slave_tx_index = 0U;
        s_slave_transmitting = READ_BIT(sr2, I2C_SR2_TRA) != 0U;
        s_slave_receiving = !s_slave_transmitting;
        SET_BIT(I2C1->CR1, I2C_CR1_ACK);
        return;
    }

    if (READ_BIT(sr1, I2C_SR1_TXE) && s_slave_transmitting) {
        uint16_t response_length = s_slave_response_length;

        I2C1->DR = (response_length > 0U)
                       ? s_slave_response[s_slave_tx_index % response_length]
                       : 0xFFU;
        s_slave_tx_index++;
        if (s_slave_tx_count < s_slave_max_tx_bytes) {
            s_slave_tx_count++;
        }
        return;
    }

    if (READ_BIT(sr1, I2C_SR1_RXNE)) {
        uint8_t byte = (uint8_t)I2C1->DR;
        s_slave_ev_isr_entries++;
        s_slave_last_sr1_seen = sr1;
        s_slave_rxne_events++;
        if (s_slave_receiving) {
            if (s_slave_frame_length < s_slave_max_rx_bytes) {
                s_slave_frame[s_slave_frame_length++] = byte;
                if (s_slave_frame_length >= s_slave_max_rx_bytes) {
                    CLEAR_BIT(I2C1->CR1, I2C_CR1_ACK);
                }
            }
        }
    }

    if (READ_BIT(sr1, I2C_SR1_STOPF)) {
        I2C1_SlaveReceiveCallback callback = s_slave_callback;
        I2C1_SlaveEventCallback event_callback = s_slave_event_callback;
        bool completed_write = s_slave_receiving;
        /* Clear STOPF: read SR1 (above), then write CR1. */
        I2C1->CR1 |= I2C_CR1_PE;
        s_slave_ev_isr_entries++;
        s_slave_last_sr1_seen = sr1;
        s_slave_stopf_events++;
        s_slave_receiving = false;
        SET_BIT(I2C1->CR1, I2C_CR1_ACK);
        if ((s_slave_frame_length > 0U) && (callback != NULL)) {
            callback((const uint8_t *)s_slave_frame, s_slave_frame_length);
        }
        if (completed_write && (event_callback != NULL)) {
            event_callback(I2C1_SLAVE_EVENT_MASTER_WRITE,
                           (const uint8_t *)s_slave_frame,
                           s_slave_frame_length);
        }
        return;
    }

    if (!READ_BIT(sr1, (I2C_SR1_ADDR | I2C_SR1_RXNE | I2C_SR1_TXE |
                       I2C_SR1_STOPF))) {
        s_slave_ev_isr_entries++;
        s_slave_last_sr1_seen = sr1;
        s_slave_unmatched_events++;
    }
}

void I2C1_ER_IRQHandler(void)
{
    if (!s_master_transaction_active) {
        uint32_t sr1 = I2C1->SR1;
        bool completed_read = s_slave_transmitting &&
                              READ_BIT(sr1, I2C_SR1_AF);
        bool bus_error = READ_BIT(sr1, I2C_SR1_BERR) != 0U;
        bool receiving = s_slave_receiving;
        uint16_t received = s_slave_frame_length;
        uint16_t transmitted = s_slave_tx_count;
        I2C1_SlaveEventCallback event_callback = s_slave_event_callback;
        /* Slave-mode path, unchanged: master routines don't own anything
         * here, so this branch clears AF/BERR/ARLO itself and resets the
         * in-progress slave receive state. */
        s_slave_er_isr_entries++;
        s_slave_er_last_sr1_seen = sr1;
        I2C1->SR1 &= ~(I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO);
        s_slave_receiving = false;
        s_slave_transmitting = false;
        s_slave_frame_length = 0U;
        SET_BIT(I2C1->CR1, I2C_CR1_ACK);
        if (completed_read && (event_callback != NULL)) {
            event_callback(I2C1_SLAVE_EVENT_MASTER_READ, NULL, transmitted);
        }
        if (bus_error && (event_callback != NULL)) {
            event_callback(I2C1_SLAVE_EVENT_BUS_ERROR,
                           receiving ? (const uint8_t *)s_slave_frame : NULL,
                           receiving ? received : 0U);
        }
        return;
    }

    /* The master routines own AF handling - I2C1_SendAddress() and
     * I2C1_Master_Transmit() both poll SR1 directly and clear AF themselves
     * (see their NACK-detection branches), so AF is deliberately left alone
     * here while a master transaction is active.
     *
     * BERR (bus error) and ARLO (arbitration lost) are a different story:
     * Foreground master routines cannot safely own clearing these flags
     * because the enabled error IRQ can preempt their polling loops. This
     * bus is genuinely multi-master (the attached
     * endpoint can also initiate transactions, e.g. an unsolicited SMBus
     * Block Write), so a real BERR/ARLO during OUR master transaction is
     * entirely possible, not a "won't happen" case. Previously, if one hit
     * while s_master_transaction_active was true, this function returned
     * without touching SR1 at all - the flag stayed set, ITERREN stayed
     * enabled, and the NVIC re-entered this handler again immediately,
     * forever: a genuine CPU livelock, not a bounded polling timeout,
     * since nothing in the foreground master code looks at BERR/ARLO
     * either. This is what caused i2c_detect/i2c_count to hang solid
     * immediately after a completed smbus_read instead of merely running
     * slowly - the very next master transaction (the scan's first probe)
     * hit it before returning even once.
     *
     * Clearing BERR/ARLO here (without touching AF) closes that gap for
     * every master routine at once. Existing NACK-reporting behavior is
     * unaffected since AF detection still belongs entirely to foreground
     * code. The extended foreground APIs inspect s_master_error_flags and
     * report BUS_ERROR or ARB_LOST; legacy APIs map those detailed results
     * back to their older I2C_Status_t values. */
    if (READ_BIT(I2C1->SR1, I2C_SR1_BERR | I2C_SR1_ARLO)) {
        s_master_error_flags |=
            READ_BIT(I2C1->SR1, I2C_SR1_BERR | I2C_SR1_ARLO);
        s_master_er_isr_entries++;
        I2C1->SR1 &= ~(I2C_SR1_BERR | I2C_SR1_ARLO);
    }
}

I2C_Status_t I2C1_Start(void)
{
    return I2C1_GenerateStart(true);
}

void I2C1_Stop(void)
{
    I2C1_RequestStop();
    I2C1_WaitForStopAndRecover();
}

I2C_Status_t I2C1_SendAddress(uint8_t dev_addr, uint8_t read)
{
    uint32_t timeout = I2C1_TIMEOUT_DEFAULT;

    I2C1->DR = (uint8_t)((dev_addr << 1) | (read ? 1U : 0U));

    while (!READ_BIT(I2C1->SR1, I2C_SR1_ADDR) && --timeout) {
        if (READ_BIT(I2C1->SR1, I2C_SR1_AF)) {
            CLEAR_BIT(I2C1->SR1, I2C_SR1_AF);
            I2C1_Stop();
            return I2C_NACK;
        }
    }
    if (timeout == 0) {
        I2C1_Stop();
        return I2C_TIMEOUT;
    }

    /* Clear ADDR flag: read SR1 then SR2 (already read SR1 above via READ_BIT) */
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    return I2C_OK;
}

static I2C_ExtendedStatus_t I2C1_MasterExtendedFault(void)
{
    if (READ_BIT(s_master_error_flags, I2C_SR1_ARLO) != 0U) {
        return I2C_EXT_STATUS_ARB_LOST;
    }
    if (READ_BIT(s_master_error_flags, I2C_SR1_BERR) != 0U) {
        return I2C_EXT_STATUS_BUS_ERROR;
    }
    return I2C_EXT_STATUS_OK;
}

I2C_ExtendedStatus_t I2C1_Master_TransmitExt(uint8_t dev_addr,
                                             const uint8_t *data,
                                             uint16_t len,
                                             uint16_t *num_written)
{
    I2C_Status_t status;
    I2C_ExtendedStatus_t extended;
    uint32_t timeout;

    if ((num_written == NULL) || ((data == NULL) && (len > 0U))) {
        return I2C_EXT_STATUS_BUS_ERROR;
    }
    *num_written = 0U;
    s_master_error_flags = 0U;
    LOG_DEBUG("I2C1 TX: start addr=0x%02X len=%u", dev_addr, (unsigned int)len);
    status = I2C1_Start();
    if (status != I2C_OK) {
        LOG_ERROR("I2C1 TX: START failed (%d)", status);
        extended = I2C1_MasterExtendedFault();
        return (extended != I2C_EXT_STATUS_OK)
                   ? extended : I2C_EXT_STATUS_BUS_LOCKED;
    }

    status = I2C1_SendAddress(dev_addr, 0);
    if (status != I2C_OK) {
        LOG_ERROR("I2C1 TX: address 0x%02X failed (%d)", dev_addr, status);
        I2C1_LogMasterFault("I2C1 TX: address phase", I2C1->SR1, I2C1->SR2);
        extended = I2C1_MasterExtendedFault();
        if (extended != I2C_EXT_STATUS_OK) {
            return extended;
        }
        return (status == I2C_NACK) ? I2C_EXT_STATUS_SLA_NACK
                                    : I2C_EXT_STATUS_BUS_LOCKED;
    }
    LOG_DEBUG("I2C1 TX: address acknowledged");

    /* A zero-byte write is the SMBus Quick Command write form: the address
     * phase itself is the operation, followed immediately by STOP. */
    if (len == 0U) {
        I2C1_Stop();
        LOG_DEBUG("I2C1 TX: zero-byte quick command complete");
        return I2C_EXT_STATUS_OK;
    }

    for (uint16_t i = 0; i < len; i++) {
        timeout = I2C1_TIMEOUT_DEFAULT;
        while (!READ_BIT(I2C1->SR1, I2C_SR1_TXE) && --timeout) {
            extended = I2C1_MasterExtendedFault();
            if (extended != I2C_EXT_STATUS_OK) {
                I2C1_Stop();
                return extended;
            }
            if (READ_BIT(I2C1->SR1, I2C_SR1_AF)) {
                I2C1_LogMasterFault("I2C1 TX: data byte NACK", I2C1->SR1, I2C1->SR2);
                CLEAR_BIT(I2C1->SR1, I2C_SR1_AF);
                I2C1_Stop();
                return I2C_EXT_STATUS_DATA_NACK;
            }
        }
        if (timeout == 0) {
            I2C1_LogMasterFault("I2C1 TX: TXE timeout", I2C1->SR1, I2C1->SR2);
            I2C1_Stop();
            return I2C_EXT_STATUS_BUS_LOCKED;
        }

        I2C1->DR = data[i];
        timeout = I2C1_TIMEOUT_DEFAULT;
        while (!READ_BIT(I2C1->SR1, I2C_SR1_BTF) && --timeout) {
            extended = I2C1_MasterExtendedFault();
            if (extended != I2C_EXT_STATUS_OK) {
                I2C1_Stop();
                return extended;
            }
            if (READ_BIT(I2C1->SR1, I2C_SR1_AF)) {
                I2C1_LogMasterFault("I2C1 TX: data byte NACK",
                                    I2C1->SR1, I2C1->SR2);
                CLEAR_BIT(I2C1->SR1, I2C_SR1_AF);
                I2C1_Stop();
                return I2C_EXT_STATUS_DATA_NACK;
            }
        }
        extended = I2C1_MasterExtendedFault();
        if (extended != I2C_EXT_STATUS_OK) {
            I2C1_Stop();
            return extended;
        }
        if (READ_BIT(I2C1->SR1, I2C_SR1_AF)) {
            I2C1_LogMasterFault("I2C1 TX: data byte NACK",
                                I2C1->SR1, I2C1->SR2);
            CLEAR_BIT(I2C1->SR1, I2C_SR1_AF);
            I2C1_Stop();
            return I2C_EXT_STATUS_DATA_NACK;
        }
        if (timeout == 0U) {
            I2C1_LogMasterFault("I2C1 TX: BTF timeout", I2C1->SR1, I2C1->SR2);
            I2C1_Stop();
            return I2C_EXT_STATUS_BUS_LOCKED;
        }
        *num_written = (uint16_t)(i + 1U);
    }

    I2C1_Stop();
    LOG_DEBUG("I2C1 TX: complete");
    return I2C_EXT_STATUS_OK;
}

I2C_Status_t I2C1_Master_Transmit(uint8_t dev_addr,
                                  const uint8_t *data,
                                  uint16_t len)
{
    uint16_t num_written;

    switch (I2C1_Master_TransmitExt(dev_addr, data, len, &num_written)) {
    case I2C_EXT_STATUS_OK:         return I2C_OK;
    case I2C_EXT_STATUS_SLA_NACK:
    case I2C_EXT_STATUS_DATA_NACK:  return I2C_NACK;
    case I2C_EXT_STATUS_ARB_LOST:   return I2C_ARBITRATION_LOST;
    case I2C_EXT_STATUS_BUS_LOCKED: return I2C_TIMEOUT;
    case I2C_EXT_STATUS_BUS_ERROR:
    case I2C_EXT_STATUS_SLA_ACK:
    case I2C_EXT_STATUS_LAST_DATA_ACK:
    default:                        return I2C_BUS_ERROR;
    }
}

I2C_Status_t I2C1_Master_Receive(uint8_t dev_addr, uint8_t *data, uint16_t len)
{
    return I2C1_ReceiveCore(dev_addr, data, len, false);
}

I2C_ExtendedStatus_t I2C1_Master_ReceiveExt(uint8_t dev_addr,
                                            uint8_t *data,
                                            uint16_t len,
                                            uint16_t *num_read)
{
    return I2C1_ReceiveCoreExt(dev_addr, data, len, false, num_read);
}

I2C_Status_t I2C1_SMBus_BlockWrite(uint8_t dev_addr,
                                    uint8_t command,
                                    const uint8_t *data,
                                    uint8_t len)
{
    uint8_t frame[2U + I2C1_SMBUS_BLOCK_MAX_LENGTH];

    if ((len == 0U) || (len > I2C1_SMBUS_BLOCK_MAX_LENGTH) || (data == NULL)) {
        return I2C_ERROR;
    }

    frame[0] = command;
    frame[1] = len;
    for (uint8_t index = 0U; index < len; index++) {
        frame[2U + index] = data[index];
    }

    return I2C1_Master_Transmit(dev_addr, frame, (uint16_t)len + 2U);
}

I2C_Status_t I2C1_Master_MemWrite(uint8_t dev_addr, uint8_t mem_addr, const uint8_t *data, uint16_t len)
{
    I2C_Status_t status;
    uint32_t timeout;

    status = I2C1_Start();
    if (status != I2C_OK) return status;

    status = I2C1_SendAddress(dev_addr, 0);
    if (status != I2C_OK) return status;

    timeout = I2C1_TIMEOUT_DEFAULT;
    while (!READ_BIT(I2C1->SR1, I2C_SR1_TXE) && --timeout) { }
    if (timeout == 0) { I2C1_Stop(); return I2C_TIMEOUT; }
    I2C1->DR = mem_addr;

    for (uint16_t i = 0; i < len; i++) {
        timeout = I2C1_TIMEOUT_DEFAULT;
        while (!READ_BIT(I2C1->SR1, I2C_SR1_TXE) && --timeout) {
            if (READ_BIT(I2C1->SR1, I2C_SR1_AF)) {
                CLEAR_BIT(I2C1->SR1, I2C_SR1_AF);
                I2C1_Stop();
                return I2C_NACK;
            }
        }
        if (timeout == 0) { I2C1_Stop(); return I2C_TIMEOUT; }

        I2C1->DR = data[i];
    }

    timeout = I2C1_TIMEOUT_DEFAULT;
    while (!READ_BIT(I2C1->SR1, I2C_SR1_BTF) && --timeout) { }
    if (timeout == 0) { I2C1_Stop(); return I2C_TIMEOUT; }

    I2C1_Stop();
    return I2C_OK;
}

I2C_Status_t I2C1_Master_MemRead(uint8_t dev_addr, uint8_t mem_addr, uint8_t *data, uint16_t len)
{
    I2C_Status_t status;
    uint32_t timeout;

    status = I2C1_Start();
    if (status != I2C_OK) return status;

    status = I2C1_SendAddress(dev_addr, 0);
    if (status != I2C_OK) return status;

    timeout = I2C1_TIMEOUT_DEFAULT;
    while (!READ_BIT(I2C1->SR1, I2C_SR1_TXE) && --timeout) { }
    if (timeout == 0) { I2C1_Stop(); return I2C_TIMEOUT; }
    I2C1->DR = mem_addr;

    timeout = I2C1_TIMEOUT_DEFAULT;
    while (!READ_BIT(I2C1->SR1, I2C_SR1_BTF) && --timeout) { }
    if (timeout == 0) { I2C1_Stop(); return I2C_TIMEOUT; }

    /* Repeated START into the read phase - do NOT wait on BUSY here,
     * we still own the bus from the write phase above. */
    return I2C1_ReceiveCore(dev_addr, data, len, true);
}

bool I2C1_IsDeviceReady(uint8_t dev_addr)
{
    I2C_Status_t status = I2C1_Start();
    if (status != I2C_OK) return false;

    status = I2C1_SendAddress(dev_addr, 0);
    I2C1_Stop();

    return (status == I2C_OK);
}

bool I2C1_ScanBus(void)
{
    /* Raw pin readback - independent of the I2C1 peripheral. Useful to see
     * what's actually sitting on the wire, no matter what I2C1 itself thinks. */
    LOG_INFO("I2C1: PB8(SCL)=%lu  PB9(SDA)=%lu",
              (unsigned long)((GPIOB->IDR >> 8) & 1U),
              (unsigned long)((GPIOB->IDR >> 9) & 1U));

    /* Confirm PB8/PB9 are actually wired to the I2C1 peripheral internally,
     * not just idle GPIO pins reading high off the pull-up. Expect MODER
     * bits[19:16]=0xA (both AF mode) and AFR1 bits[7:0]=0x44 (both AF4). */
    LOG_INFO("I2C1: MODER[19:16]=0x%lX (expect 0xA)  AFR1[7:0]=0x%lX (expect 0x44)",
              (unsigned long)((GPIOB->MODER >> 16) & 0xFU),
              (unsigned long)(GPIOB->AFR[1] & 0xFFU));

    LOG_INFO("I2C1: scanning bus (0x03-0x77)...");
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        I2C_Status_t start_status = I2C1_Start();

        if (start_status == I2C_TIMEOUT) {
            /* Bus never went idle - SDA/SCL likely stuck low. Almost always
             * missing/insufficient pull-ups or a wiring/ground issue, not a
             * "device not present" situation. No point scanning further. */
            LOG_INFO("I2C1: bus busy/stuck (timed out waiting for idle at 0x%02X)", addr);
            LOG_INFO("I2C1: check external pull-ups on SDA/SCL and common GND");
            return false;
        }

        /* DEBUG: log exact status for the first 3 addresses of every scan,
         * instead of inferring what happened from the aggregate result. */
        if (addr <= 0x05) {
            LOG_INFO("I2C1: [0x%02X] Start=%s  SR1=0x%04lX  SR2=0x%04lX",
                      addr,
                      (start_status == I2C_OK) ? "OK" : "FAIL",
                      (unsigned long)I2C1->SR1,
                      (unsigned long)I2C1->SR2);
        }

        I2C_Status_t addr_status = I2C1_SendAddress(addr, 0);

        if (addr <= 0x05) {
            const char *s = (addr_status == I2C_OK) ? "OK (ACK)" :
                             (addr_status == I2C_NACK) ? "NACK" :
                             (addr_status == I2C_TIMEOUT) ? "TIMEOUT" : "?";
            LOG_INFO("I2C1: [0x%02X] SendAddress=%s  SR1=0x%04lX  SR2=0x%04lX",
                      addr, s,
                      (unsigned long)I2C1->SR1,
                      (unsigned long)I2C1->SR2);
        }

        I2C1_Stop();

        if (addr_status == I2C_OK) {
            LOG_INFO("I2C1: device found at 0x%02X", addr);
            LOG_INFO("I2C1: stopping scan at first device");
            return true;
        }
        /* I2C_NACK here just means "nobody answered this address" - normal,
         * not logged per-address to avoid spamming 117 lines every scan. */
    }

    LOG_INFO("I2C1: bus is idle/responsive but no devices ACKed (0 found)");

    return false;
}
