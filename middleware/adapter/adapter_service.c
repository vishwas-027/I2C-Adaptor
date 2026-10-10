#include "adapter_service.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "../mctp/mctp_smbus.h"

#include <string.h>

static SemaphoreHandle_t s_i2c_mutex;
static volatile bool s_connected;
static Mctp s_mctp;
static MctpSmbus s_mctp_smbus;
static volatile uint8_t s_async_pending_flags;
static volatile uint8_t s_i2c_slave_address = ADAPTER_LOCAL_SMBUS_ADDRESS;
static uint16_t s_i2c_slave_max_tx_bytes = ADAPTER_I2C_SLAVE_DEFAULT_MAX_TX;
static QueueHandle_t s_i2c_slave_rx_queue;

#define ADAPTER_MCTP_PACKET_MTU 64U
#define ADAPTER_I2C_SLAVE_RX_QUEUE_DEPTH 4U

typedef struct {
    I2C_ExtendedStatus_t status;
    uint16_t length;
    uint8_t address;
    uint8_t data[I2C1_SLAVE_RX_MAX_LENGTH];
} AdapterI2cSlaveRxFrame;

static void set_i2c_connection_state(I2C_Status_t status)
{
    s_connected = (status == I2C_OK);
}

static void adapter_i2c_slave_event(I2C1_SlaveEvent_t event,
                                    const uint8_t *frame,
                                    uint16_t length)
{
    if ((event == I2C1_SLAVE_EVENT_MASTER_WRITE) ||
        (event == I2C1_SLAVE_EVENT_BUS_ERROR)) {
        AdapterI2cSlaveRxFrame received;
        BaseType_t task_woken = pdFALSE;

        received.status = (event == I2C1_SLAVE_EVENT_BUS_ERROR)
                              ? I2C_EXT_STATUS_BUS_ERROR
                              : I2C_EXT_STATUS_OK;
        received.address = s_i2c_slave_address;
        received.length = (length <= I2C1_SLAVE_RX_MAX_LENGTH)
                              ? length : I2C1_SLAVE_RX_MAX_LENGTH;
        for (uint16_t index = 0U; index < received.length; index++) {
            received.data[index] = frame[index];
        }
        (void)xQueueSendFromISR(s_i2c_slave_rx_queue, &received,
                                &task_woken);
        s_async_pending_flags |= ADAPTER_ASYNC_I2C_READ;
        portYIELD_FROM_ISR(task_woken);
    } else if (event == I2C1_SLAVE_EVENT_MASTER_READ) {
        s_async_pending_flags |= ADAPTER_ASYNC_I2C_WRITE;
    }
}

void adapter_service_init(void)
{
    s_i2c_mutex = xSemaphoreCreateMutex();
    configASSERT(s_i2c_mutex != NULL);
    s_i2c_slave_rx_queue = xQueueCreate(ADAPTER_I2C_SLAVE_RX_QUEUE_DEPTH,
                                        sizeof(AdapterI2cSlaveRxFrame));
    configASSERT(s_i2c_slave_rx_queue != NULL);
    s_connected = false;
    s_async_pending_flags = ADAPTER_ASYNC_NO_DATA;

    I2C1_Init(I2C_SPEED_STANDARD_100K);
    I2C1_SetSlaveEventCallback(adapter_i2c_slave_event);
    configASSERT(mctp_smbus_i2c1_enable_receive(
                     ADAPTER_LOCAL_SMBUS_ADDRESS) == MCTP_OK);
    configASSERT(mctp_smbus_init(&s_mctp, &s_mctp_smbus,
                                 ADAPTER_LOCAL_SMBUS_ADDRESS,
                                 mctp_smbus_i2c1_write,
                                 mctp_smbus_i2c1_read,
                                 NULL,
                                 MCTP_TIMEOUT_DEFAULT_MS) == MCTP_OK);
}

bool adapter_service_is_connected(void)
{
    return s_connected;
}

I2C_Status_t adapter_i2c_bitrate(uint32_t bitrate_hz)
{
    I2C_Status_t status;

    if ((bitrate_hz != (uint32_t)I2C_SPEED_STANDARD_100K) &&
        (bitrate_hz != (uint32_t)I2C_SPEED_FAST_400K)) {
        return I2C_ERROR;
    }
    if (xSemaphoreTake(s_i2c_mutex, portMAX_DELAY) != pdTRUE) {
        return I2C_ERROR;
    }
    status = I2C1_SetBitrate(bitrate_hz);
    (void)xSemaphoreGive(s_i2c_mutex);
    return status;
}

uint32_t adapter_i2c_get_bitrate(void)
{
    return I2C1_GetBitrate();
}

I2C_Status_t adapter_i2c_slave_enable(uint8_t slave_address,
                                      uint16_t max_tx_bytes,
                                      uint16_t max_rx_bytes,
                                      uint16_t *applied_max_tx_bytes,
                                      uint16_t *applied_max_rx_bytes)
{
    I2C_Status_t status;
    uint16_t effective_tx = (max_tx_bytes == 0U)
                                ? ADAPTER_I2C_SLAVE_DEFAULT_MAX_TX
                                : max_tx_bytes;
    uint16_t effective_rx = (max_rx_bytes == 0U)
                                ? ADAPTER_I2C_SLAVE_DEFAULT_MAX_RX
                                : max_rx_bytes;

    if ((slave_address > 0x7FU) ||
        (effective_tx > ADAPTER_I2C_SLAVE_DEFAULT_MAX_TX) ||
        (effective_rx > ADAPTER_I2C_SLAVE_DEFAULT_MAX_RX) ||
        (applied_max_tx_bytes == NULL) ||
        (applied_max_rx_bytes == NULL)) {
        return I2C_ERROR;
    }
    if (xSemaphoreTake(s_i2c_mutex, portMAX_DELAY) != pdTRUE) {
        return I2C_ERROR;
    }
    status = mctp_smbus_i2c1_configure_receive(
        slave_address, effective_tx, effective_rx);
    if (status == I2C_OK) {
        s_mctp_smbus.local_smbus_address = slave_address;
        s_i2c_slave_address = slave_address;
        s_i2c_slave_max_tx_bytes = effective_tx;
        s_async_pending_flags = ADAPTER_ASYNC_NO_DATA;
        (void)xQueueReset(s_i2c_slave_rx_queue);
        *applied_max_tx_bytes = effective_tx;
        *applied_max_rx_bytes = effective_rx;
    }
    (void)xSemaphoreGive(s_i2c_mutex);
    return status;
}

I2C_Status_t adapter_i2c_slave_write(const uint8_t *data,
                                     uint16_t length,
                                     uint16_t *accepted_length)
{
    I2C_Status_t status;
    uint16_t accepted;

    if ((data == NULL) || (length == 0U) ||
        (length > ADAPTER_I2C_SLAVE_DEFAULT_MAX_TX) ||
        (accepted_length == NULL)) {
        return I2C_ERROR;
    }
    *accepted_length = 0U;
    if (xSemaphoreTake(s_i2c_mutex, portMAX_DELAY) != pdTRUE) {
        return I2C_ERROR;
    }
    accepted = (length < s_i2c_slave_max_tx_bytes)
                   ? length : s_i2c_slave_max_tx_bytes;
    taskENTER_CRITICAL();
    status = I2C1_SetSlaveResponse(data, accepted);
    taskEXIT_CRITICAL();
    if (status == I2C_OK) {
        *accepted_length = accepted;
    }
    (void)xSemaphoreGive(s_i2c_mutex);
    return status;
}

uint8_t adapter_async_poll(void)
{
    return s_async_pending_flags;
}

I2C_Status_t adapter_i2c_slave_read_ext(
    uint16_t buffer_size,
    uint8_t *address,
    uint8_t *data,
    uint16_t *num_read,
    I2C_ExtendedStatus_t *transaction_status)
{
    AdapterI2cSlaveRxFrame received;
    uint16_t copy_length;

    if (address != NULL) {
        *address = 0U;
    }
    if (num_read != NULL) {
        *num_read = 0U;
    }
    if (transaction_status != NULL) {
        *transaction_status = I2C_EXT_STATUS_OK;
    }
    if ((buffer_size == 0U) ||
        (buffer_size > ADAPTER_I2C_SLAVE_DEFAULT_MAX_RX) ||
        (address == NULL) || (data == NULL) || (num_read == NULL) ||
        (transaction_status == NULL) || (s_i2c_slave_rx_queue == NULL)) {
        return I2C_ERROR;
    }
    if (xQueueReceive(s_i2c_slave_rx_queue, &received,
                      pdMS_TO_TICKS(ADAPTER_I2C_SLAVE_READ_TIMEOUT_MS)) !=
        pdTRUE) {
        return I2C_TIMEOUT;
    }

    copy_length = (received.length < buffer_size)
                      ? received.length : buffer_size;
    for (uint16_t index = 0U; index < copy_length; index++) {
        data[index] = received.data[index];
    }
    *address = received.address;
    *num_read = copy_length;
    *transaction_status = received.status;

    taskENTER_CRITICAL();
    if (uxQueueMessagesWaiting(s_i2c_slave_rx_queue) == 0U) {
        s_async_pending_flags &= (uint8_t)~ADAPTER_ASYNC_I2C_READ;
    }
    taskEXIT_CRITICAL();
    return I2C_OK;
}

I2C_Status_t adapter_i2c_write(uint8_t address,
                               uint16_t flags,
                               const uint8_t *data,
                               uint16_t length)
{
    uint16_t num_written;

    switch (adapter_i2c_write_ext(address, flags, data, length,
                                  &num_written)) {
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

I2C_ExtendedStatus_t adapter_i2c_write_ext(uint8_t address,
                                           uint16_t flags,
                                           const uint8_t *data,
                                           uint16_t length,
                                           uint16_t *num_written)
{
    I2C_ExtendedStatus_t status;

    if (num_written != NULL) {
        *num_written = 0U;
    }
    if ((address > 0x7FU) || (flags != ADAPTER_I2C_NO_FLAGS) ||
        (length > ADAPTER_I2C_WRITE_MAX_LENGTH) ||
        ((length > 0U) && (data == NULL)) || (num_written == NULL)) {
        return I2C_EXT_STATUS_BUS_ERROR;
    }
    if (xSemaphoreTake(s_i2c_mutex, portMAX_DELAY) != pdTRUE) {
        return I2C_EXT_STATUS_BUS_ERROR;
    }
    status = I2C1_Master_TransmitExt(address, data, length, num_written);
    (void)xSemaphoreGive(s_i2c_mutex);
    set_i2c_connection_state((status == I2C_EXT_STATUS_OK)
                                 ? I2C_OK : I2C_ERROR);
    return status;
}

I2C_Status_t adapter_smbus_read(uint8_t address,
                                uint8_t *data,
                                uint8_t length)
{
    uint16_t num_read;

    if ((address > 0x7FU) || (data == NULL) || (length == 0U) ||
        (length > I2C1_SMBUS_BLOCK_MAX_LENGTH)) {
        return I2C_ERROR;
    }
    switch (adapter_i2c_read_ext(address, ADAPTER_I2C_NO_FLAGS, data,
                                 length, &num_read)) {
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

I2C_ExtendedStatus_t adapter_i2c_read_ext(uint8_t address,
                                          uint16_t flags,
                                          uint8_t *data,
                                          uint16_t length,
                                          uint16_t *num_read)
{
    I2C_ExtendedStatus_t status;

    if (num_read != NULL) {
        *num_read = 0U;
    }
    if ((address > 0x7FU) || (flags != ADAPTER_I2C_NO_FLAGS) ||
        (data == NULL) || (length == 0U) ||
        (length > ADAPTER_I2C_READ_MAX_LENGTH) || (num_read == NULL)) {
        return I2C_EXT_STATUS_BUS_ERROR;
    }
    if (xSemaphoreTake(s_i2c_mutex, portMAX_DELAY) != pdTRUE) {
        return I2C_EXT_STATUS_BUS_ERROR;
    }
    status = I2C1_Master_ReceiveExt(address, data, length, num_read);
    (void)xSemaphoreGive(s_i2c_mutex);
    set_i2c_connection_state((status == I2C_EXT_STATUS_OK)
                                 ? I2C_OK : I2C_ERROR);
    return status;
}

I2C_Status_t adapter_smbus_write(uint8_t address,
                                 uint8_t command,
                                 const uint8_t *data,
                                 uint8_t length)
{
    I2C_Status_t status;

    if ((address > 0x7FU) || (data == NULL) || (length == 0U) ||
        (length > I2C1_SMBUS_BLOCK_MAX_LENGTH)) {
        return I2C_ERROR;
    }
    if (xSemaphoreTake(s_i2c_mutex, portMAX_DELAY) != pdTRUE) {
        return I2C_ERROR;
    }
    status = I2C1_SMBus_BlockWrite(address, command, data, length);
    (void)xSemaphoreGive(s_i2c_mutex);
    set_i2c_connection_state(status);
    return status;
}

I2C_Status_t adapter_i2c_detect(uint8_t address_bitmap[16],
                                uint8_t *device_count,
                                AdapterScanCancelFn cancel,
                                void *cancel_context,
                                bool *cancelled)
{
    uint8_t count = 0U;
    I2C_Status_t scan_status = I2C_OK;

    if ((address_bitmap == NULL) || (device_count == NULL)) {
        return I2C_ERROR;
    }
    memset(address_bitmap, 0, 16U);
    *device_count = 0U;
    if (cancelled != NULL) {
        *cancelled = false;
    }
    if (xSemaphoreTake(s_i2c_mutex, portMAX_DELAY) != pdTRUE) {
        return I2C_ERROR;
    }
    for (uint8_t address = 0x03U; address <= 0x77U; address++) {
        I2C_Status_t status;

        if ((cancel != NULL) && cancel(cancel_context)) {
            if (cancelled != NULL) {
                *cancelled = true;
            }
            break;
        }
        status = I2C1_Start();
        if (status != I2C_OK) {
            /* A START timeout normally means BUSY is stuck because the bus
             * is held low. Repeating it for every address only turns one
             * electrical fault into a minute-long USB command. */
            scan_status = status;
            break;
        }
        status = I2C1_SendAddress(address, 0U);
        I2C1_Stop();
        if (status == I2C_OK) {
            address_bitmap[address >> 3U] |= (uint8_t)(1U << (address & 7U));
            count++;
        }
    }
    (void)xSemaphoreGive(s_i2c_mutex);
    *device_count = count;
    if (scan_status != I2C_OK) {
        s_connected = false;
    } else if ((cancelled == NULL) || !*cancelled) {
        s_connected = (count > 0U);
    }
    return scan_status;
}

MctpStatus adapter_mctp_send_receive(uint8_t destination_smbus_address,
                                     const uint8_t *request,
                                     uint16_t request_length,
                                     uint32_t timeout_ms,
                                     MctpPacket *response)
{
    MctpStatus status;

    if ((destination_smbus_address > 0x7FU) || (request == NULL) ||
        (request_length < MCTP_HEADER_SIZE) ||
        (request_length > MCTP_PACKET_MAX_SIZE) || (response == NULL)) {
        return MCTP_ERROR_INVALID_ARGUMENT;
    }
    if (xSemaphoreTake(s_i2c_mutex, portMAX_DELAY) != pdTRUE) {
        return MCTP_ERROR_TRANSPORT;
    }
    /* Raw I2C slave capture and MCTP reception observe the same Block Writes
     * through separate queues. Drop frames left in the MCTP queue by a raw
     * capture before associating the next packet with this request. */
    status = mctp_smbus_i2c1_flush_receive();
    if ((status == MCTP_OK) &&
        (request_length <= ADAPTER_MCTP_PACKET_MTU)) {
        status = mctp_send(&s_mctp, destination_smbus_address,
                           request, request_length);
    } else if (status == MCTP_OK) {
        uint8_t fragment[ADAPTER_MCTP_PACKET_MTU];
        uint16_t message_offset = 0U;
        uint8_t sequence = 0U;

        status = MCTP_OK;
        while ((status == MCTP_OK) &&
               (message_offset < (uint16_t)(request_length - MCTP_HEADER_SIZE))) {
            uint16_t remaining = (uint16_t)(request_length - MCTP_HEADER_SIZE -
                                            message_offset);
            uint16_t payload_length =
                (remaining > (ADAPTER_MCTP_PACKET_MTU - MCTP_HEADER_SIZE))
                    ? (ADAPTER_MCTP_PACKET_MTU - MCTP_HEADER_SIZE)
                    : remaining;
            bool som = (message_offset == 0U);
            bool eom = (payload_length == remaining);

            fragment[0] = request[0];
            fragment[1] = request[1];
            fragment[2] = request[2];
            fragment[3] = (uint8_t)((som ? 0x80U : 0U) |
                                    (eom ? 0x40U : 0U) |
                                    ((sequence & 0x03U) << 4U) |
                                    (request[3] & 0x0FU));
            memcpy(&fragment[MCTP_HEADER_SIZE],
                   &request[MCTP_HEADER_SIZE + message_offset],
                   payload_length);
            status = mctp_send(&s_mctp, destination_smbus_address, fragment,
                               (uint16_t)(MCTP_HEADER_SIZE + payload_length));
            message_offset = (uint16_t)(message_offset + payload_length);
            sequence++;
        }
    }
    if (status == MCTP_OK) {
        status = mctp_receive(&s_mctp, response, timeout_ms);
    }
    (void)xSemaphoreGive(s_i2c_mutex);
    s_connected = (status == MCTP_OK);
    return status;
}
