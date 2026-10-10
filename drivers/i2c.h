/*
 * i2c.h
 *
 * Bare-metal I2C1 master driver for STM32F411CEU6 (BlackPill).
 * No HAL / no CMSIS - direct register access only.
 *
 * Pin mapping (fixed for this driver):
 *   PB8 -> I2C1_SCL (AF4, open-drain)
 *   PB9 -> I2C1_SDA (AF4, open-drain)
 *
 * Timing is derived from SystemClock.pclk1, which the board clock setup
 * updates after switching the STM32F411 to its 84 MHz PLL configuration.
 *
 * Blocking / polling implementation (no IRQ, no DMA) - intentional for
 * this stage. See "Next steps" notes for the interrupt-driven upgrade path.
 */

#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f411.h"

/* Default timeout, expressed as busy-wait loop iterations (not milliseconds).
 * The driver is polling-only; callers must serialize transactions themselves. */
#define I2C1_TIMEOUT_DEFAULT   100000U
#define I2C1_SMBUS_BLOCK_MAX_LENGTH 32U
#define I2C1_SLAVE_TX_MAX_LENGTH    256U
#define I2C1_SLAVE_RX_MAX_LENGTH    258U

typedef enum {
    I2C_OK = 0,
    I2C_ERROR,
    I2C_TIMEOUT,
    I2C_NACK,
    I2C_BUS_ERROR,
    I2C_ARBITRATION_LOST,
    I2C_BUSY
} I2C_Status_t;

/* Numeric values intentionally match the Aardvark extended I2C status API. */
typedef enum {
    I2C_EXT_STATUS_OK = 0,
    I2C_EXT_STATUS_BUS_ERROR = 1,
    I2C_EXT_STATUS_SLA_ACK = 2,
    I2C_EXT_STATUS_SLA_NACK = 3,
    I2C_EXT_STATUS_DATA_NACK = 4,
    I2C_EXT_STATUS_ARB_LOST = 5,
    I2C_EXT_STATUS_BUS_LOCKED = 6,
    I2C_EXT_STATUS_LAST_DATA_ACK = 7
} I2C_ExtendedStatus_t;

typedef enum {
    I2C_SPEED_STANDARD_100K = 100000U,
    I2C_SPEED_FAST_400K     = 400000U
} I2C_Speed_t;

/* The callback runs in I2C1_EV IRQ context. The frame starts with the SMBus
 * command byte and includes all bytes through the PEC. */
typedef void (*I2C1_SlaveReceiveCallback)(const uint8_t *frame, uint16_t length);

typedef enum {
    I2C1_SLAVE_EVENT_MASTER_WRITE = 1U,
    I2C1_SLAVE_EVENT_MASTER_READ = 2U,
    I2C1_SLAVE_EVENT_BUS_ERROR = 3U
} I2C1_SlaveEvent_t;

/* Monitor callback for completed addressed-slave transactions. It runs in
 * IRQ context. MASTER_WRITE supplies the received frame; MASTER_READ reports
 * how many bytes the slave placed on the bus; BUS_ERROR supplies any partial
 * receive frame that existed when the error occurred. */
typedef void (*I2C1_SlaveEventCallback)(I2C1_SlaveEvent_t event,
                                        const uint8_t *frame,
                                        uint16_t length);

/* Configure PB8/PB9 and I2C1 timing for the requested bus speed. */
void I2C1_Init(I2C_Speed_t speed);

/* Change/read the runtime bus bitrate. Only 100 kHz and 400 kHz are
 * supported. The setter refuses to reset I2C1 while the bus is active. */
I2C_Status_t I2C1_SetBitrate(uint32_t bitrate_hz);
uint32_t I2C1_GetBitrate(void);

/* Keep I2C1 available as a multi-master bus owner while also listening as an
 * addressed slave. This is required to receive MCTP SMBus Block Writes. */
bool I2C1_EnableSlaveReceive(uint8_t own_address,
                             I2C1_SlaveReceiveCallback callback);

/* Configure the addressed-slave listener and per-transaction limits. The
 * caller must pass resolved, non-zero limits within the constants above. */
I2C_Status_t I2C1_ConfigureSlave(uint8_t own_address,
                                 uint16_t max_tx_bytes,
                                 uint16_t max_rx_bytes,
                                 I2C1_SlaveReceiveCallback callback);
/* Preload the cyclic response returned when an external controller reads this
 * device. The caller must prevent the slave ISR from running during update. */
I2C_Status_t I2C1_SetSlaveResponse(const uint8_t *data, uint16_t length);
void I2C1_SetSlaveEventCallback(I2C1_SlaveEventCallback callback);

/* Diagnostics for the slave-mode side of I2C1_EV_IRQHandler/ER_IRQHandler.
 * Call I2C1_ResetSlaveEventCounters() right before waiting for an incoming
 * response so the counts reflect exactly that wait window; on a timeout,
 * read the others to see whether any slave-mode activity occurred at all,
 * and which event last arrived. See the counters' declaration comment in
 * i2c.c for how to read the pattern. */
void I2C1_ResetSlaveEventCounters(void);
uint32_t I2C1_GetSlaveEvIsrEntries(void);
uint32_t I2C1_GetSlaveAddrEvents(void);
uint32_t I2C1_GetSlaveRxneEvents(void);
uint32_t I2C1_GetSlaveStopfEvents(void);
uint32_t I2C1_GetSlaveUnmatchedEvents(void);
uint32_t I2C1_GetSlaveLastSr1Seen(void);
uint32_t I2C1_GetSlaveErIsrEntries(void);
uint32_t I2C1_GetSlaveErLastSr1Seen(void);
/* Count of I2C1_ER_IRQHandler entries that saw BERR or ARLO while a master
 * transaction was in progress. Should read 0 in normal operation; any
 * nonzero value confirms a real bus-error/arbitration-loss event fired
 * during a master transfer - previously the exact condition that livelocked
 * the CPU (see I2C1_ER_IRQHandler() in i2c.c), now just counted and
 * cleared. */
uint32_t I2C1_GetMasterErIsrEntries(void);

/* Direct register snapshot (CR1, OAR1, SR2), not software-side inference -
 * call right when listening begins, to see definitively whether PE/ACK are
 * set and OAR1 holds the expected address, instead of assuming the driver's
 * bookkeeping matches hardware reality. Any pointer may be NULL to skip it. */
void I2C1_GetSlaveModeRegisters(uint32_t *cr1, uint32_t *oar1, uint32_t *sr2);

/* Low-level primitives exposed for diagnostics and custom transactions. */
I2C_Status_t I2C1_Start(void);
void         I2C1_Stop(void);
I2C_Status_t I2C1_SendAddress(uint8_t dev_addr, uint8_t read);

/* Complete master transactions. Device addresses are unshifted 7-bit values. */
I2C_Status_t I2C1_Master_Transmit(uint8_t dev_addr, const uint8_t *data, uint16_t len);
I2C_ExtendedStatus_t I2C1_Master_TransmitExt(uint8_t dev_addr,
                                             const uint8_t *data,
                                             uint16_t len,
                                             uint16_t *num_written);
I2C_Status_t I2C1_Master_Receive(uint8_t dev_addr, uint8_t *data, uint16_t len);
I2C_ExtendedStatus_t I2C1_Master_ReceiveExt(uint8_t dev_addr,
                                            uint8_t *data,
                                            uint16_t len,
                                            uint16_t *num_read);

/* SMBus Block Write: command byte, byte-count, then 1 to 32 data bytes.
 * Packet Error Checking (PEC) is not included. */
I2C_Status_t I2C1_SMBus_BlockWrite(uint8_t dev_addr,
                                    uint8_t command,
                                    const uint8_t *data,
                                    uint8_t len);

/* Register-style access for peripherals that expect a leading register byte. */
I2C_Status_t I2C1_Master_MemWrite(uint8_t dev_addr, uint8_t mem_addr, const uint8_t *data, uint16_t len);
I2C_Status_t I2C1_Master_MemRead(uint8_t dev_addr, uint8_t mem_addr, uint8_t *data, uint16_t len);

/* Probe or scan for 7-bit devices. Scanning stops at the first ACK. */
bool I2C1_IsDeviceReady(uint8_t dev_addr);
/* Scans until the first responding 7-bit address. Returns true immediately
 * when one is found, or false if no device ACKs. */
bool I2C1_ScanBus(void);

#endif /* I2C_H */
