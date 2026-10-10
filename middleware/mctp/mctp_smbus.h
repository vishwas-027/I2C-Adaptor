/* SMBus/I2C transport binding for the generic MCTP packet API. */
#ifndef MCTP_SMBUS_H
#define MCTP_SMBUS_H

#include "mctp.h"
#include "i2c.h"

#define MCTP_SMBUS_COMMAND_CODE  0x0FU
#define MCTP_SMBUS_SOURCE_RW_BIT 0x01U

/* The application supplies the receive side because an SMBus MCTP response
 * is an unsolicited Block Write from the responding bus master. On STM32 it
 * should normally dequeue packets captured by an I2C slave/event ISR. */
typedef MctpStatus (*MctpSmbusWriteFn)(void *context,
                                       uint8_t destination_smbus_address,
                                       const uint8_t *frame,
                                       uint16_t frame_length);
typedef MctpStatus (*MctpSmbusReadFn)(void *context,
                                      uint8_t *frame,
                                      uint16_t *frame_length,
                                      uint16_t frame_capacity,
                                      uint32_t timeout_ms);

typedef struct {
    uint8_t local_smbus_address;
    MctpSmbusWriteFn write;
    MctpSmbusReadFn read;
    void *io_context;
} MctpSmbus;

/* Initialise an MCTP instance over an SMBus binding. local_smbus_address is a
 * 7-bit address. read() receives the bytes after SMBus byte-count, including
 * source-slave-address and PEC, exactly as captured from a Block Write. */
MctpStatus mctp_smbus_init(Mctp *mctp,
                           MctpSmbus *smbus,
                           uint8_t local_smbus_address,
                           MctpSmbusWriteFn write,
                           MctpSmbusReadFn read,
                           void *io_context,
                           uint32_t timeout_ms);

/* Standard PEC (CRC-8 polynomial x^8+x^2+x+1, initial value zero). */
uint8_t mctp_smbus_pec(const uint8_t *bytes, uint16_t length);

/* Ready-to-use I2C1 write adapter. It sends an MCTP SMBus frame through the
 * existing master driver. Reception still requires a supplied slave RX queue. */
MctpStatus mctp_smbus_i2c1_write(void *context,
                                 uint8_t destination_smbus_address,
                                 const uint8_t *frame,
                                 uint16_t frame_length);

/* Configure I2C1 as the MCTP requester's addressed slave endpoint and use an
 * ISR-to-task queue as the receive backend. Call before mctp_smbus_init(),
 * then pass mctp_smbus_i2c1_read as its read function. */
MctpStatus mctp_smbus_i2c1_enable_receive(uint8_t local_smbus_address);
I2C_Status_t mctp_smbus_i2c1_configure_receive(uint8_t local_smbus_address,
                                                uint16_t max_tx_bytes,
                                                uint16_t max_rx_bytes);
/* Discard complete frames captured before a new request/response exchange.
 * Raw slave-capture users receive the same I2C writes through a separate
 * queue, so their frames must not be mistaken for a later MCTP response. */
MctpStatus mctp_smbus_i2c1_flush_receive(void);
MctpStatus mctp_smbus_i2c1_read(void *context,
                                uint8_t *frame,
                                uint16_t *frame_length,
                                uint16_t frame_capacity,
                                uint32_t timeout_ms);

#endif /* MCTP_SMBUS_H */
