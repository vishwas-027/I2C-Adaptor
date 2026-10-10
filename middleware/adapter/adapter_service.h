#ifndef ADAPTER_SERVICE_H
#define ADAPTER_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "../../drivers/i2c.h"
#include "../mctp/mctp.h"

#define ADAPTER_LOCAL_SMBUS_ADDRESS 0x20U
#define ADAPTER_I2C_SLAVE_DEFAULT_MAX_TX I2C1_SLAVE_TX_MAX_LENGTH
#define ADAPTER_I2C_SLAVE_DEFAULT_MAX_RX I2C1_SLAVE_RX_MAX_LENGTH
#define ADAPTER_I2C_NO_FLAGS             0U
#define ADAPTER_I2C_WRITE_MAX_LENGTH   507U
#define ADAPTER_I2C_READ_MAX_LENGTH    509U
#define ADAPTER_I2C_SLAVE_READ_TIMEOUT_MS 500U

typedef enum {
    ADAPTER_ASYNC_NO_DATA = 0x00U,
    ADAPTER_ASYNC_I2C_READ = 0x01U,
    ADAPTER_ASYNC_I2C_WRITE = 0x02U
} AdapterAsyncFlags;

void adapter_service_init(void);
bool adapter_service_is_connected(void);

I2C_Status_t adapter_i2c_bitrate(uint32_t bitrate_hz);
uint32_t adapter_i2c_get_bitrate(void);
I2C_Status_t adapter_i2c_slave_enable(uint8_t slave_address,
                                      uint16_t max_tx_bytes,
                                      uint16_t max_rx_bytes,
                                      uint16_t *applied_max_tx_bytes,
                                      uint16_t *applied_max_rx_bytes);
/* Non-consuming Aardvark-style pending-event query. Flags remain set until
 * the matching slave data/statistics API consumes them; re-enabling slave
 * mode also starts a fresh event state. */
uint8_t adapter_async_poll(void);
I2C_Status_t adapter_i2c_slave_read_ext(
    uint16_t buffer_size,
    uint8_t *address,
    uint8_t *data,
    uint16_t *num_read,
    I2C_ExtendedStatus_t *transaction_status);
I2C_Status_t adapter_i2c_slave_write(const uint8_t *data,
                                     uint16_t length,
                                     uint16_t *accepted_length);

I2C_Status_t adapter_i2c_write(uint8_t address,
                               uint16_t flags,
                               const uint8_t *data,
                               uint16_t length);
I2C_ExtendedStatus_t adapter_i2c_write_ext(uint8_t address,
                                           uint16_t flags,
                                           const uint8_t *data,
                                           uint16_t length,
                                           uint16_t *num_written);
I2C_ExtendedStatus_t adapter_i2c_read_ext(uint8_t address,
                                          uint16_t flags,
                                          uint8_t *data,
                                          uint16_t length,
                                          uint16_t *num_read);

I2C_Status_t adapter_smbus_read(uint8_t address,
                                uint8_t *data,
                                uint8_t length);
I2C_Status_t adapter_smbus_write(uint8_t address,
                                 uint8_t command,
                                 const uint8_t *data,
                                 uint8_t length);
typedef bool (*AdapterScanCancelFn)(void *context);
I2C_Status_t adapter_i2c_detect(uint8_t address_bitmap[16],
                                uint8_t *device_count,
                                AdapterScanCancelFn cancel,
                                void *cancel_context,
                                bool *cancelled);

MctpStatus adapter_mctp_send_receive(uint8_t destination_smbus_address,
                                     const uint8_t *request,
                                     uint16_t request_length,
                                     uint32_t timeout_ms,
                                     MctpPacket *response);

#endif /* ADAPTER_SERVICE_H */
