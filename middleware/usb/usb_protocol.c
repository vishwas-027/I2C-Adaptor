#include "usb_protocol.h"

#include "logger.h"
#include "tusb.h"
#include "FreeRTOS.h"
#include "task.h"

#include "../adapter/adapter_service.h"
#include "../../protocol/usb_protocol.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static uint8_t s_rx_message[USB_PROTOCOL_MAX_MESSAGE];
static uint16_t s_rx_length;
static uint16_t s_rx_expected_length;

static uint8_t s_tx_message[USB_PROTOCOL_MAX_MESSAGE];
static uint16_t s_tx_length;
static uint16_t s_tx_offset;
static bool s_tx_active;
static uint8_t s_adapter_response[USB_PROTOCOL_MAX_PAYLOAD];
static bool s_async_poll_active;
static bool s_async_poll_infinite;
static uint32_t s_async_poll_sequence;
static TickType_t s_async_poll_started;
static TickType_t s_async_poll_timeout_ticks;

static uint16_t load_le16(const uint8_t *bytes)
{
    return (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U);
}

static uint32_t load_le32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] |
           ((uint32_t)bytes[1] << 8U) |
           ((uint32_t)bytes[2] << 16U) |
           ((uint32_t)bytes[3] << 24U);
}

static void store_le16(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8U);
}

static void store_le32(uint8_t *bytes, uint32_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8U);
    bytes[2] = (uint8_t)(value >> 16U);
    bytes[3] = (uint8_t)(value >> 24U);
}

static void reset_rx(void)
{
    s_rx_length = 0U;
    s_rx_expected_length = USB_PROTOCOL_HEADER_SIZE;
}

static bool begin_response(uint16_t opcode,
                           uint32_t sequence,
                           UsbStatus status,
                           const uint8_t *payload,
                           uint16_t payload_length)
{
    if (s_tx_active || (payload_length > USB_PROTOCOL_MAX_PAYLOAD) ||
        ((payload_length > 0U) && (payload == NULL))) {
        return false;
    }

    store_le32(&s_tx_message[USB_HEADER_MAGIC_OFFSET], USB_PROTOCOL_MAGIC);
    s_tx_message[USB_HEADER_VERSION_OFFSET] = USB_PROTOCOL_VERSION;
    s_tx_message[USB_HEADER_TYPE_OFFSET] = USB_MESSAGE_RESPONSE;
    store_le16(&s_tx_message[USB_HEADER_OPCODE_OFFSET], opcode);
    store_le32(&s_tx_message[USB_HEADER_SEQUENCE_OFFSET], sequence);
    store_le16(&s_tx_message[USB_HEADER_PAYLOAD_LENGTH_OFFSET], payload_length);
    store_le16(&s_tx_message[USB_HEADER_STATUS_OFFSET], (uint16_t)status);

    if (payload_length > 0U) {
        memcpy(&s_tx_message[USB_PROTOCOL_HEADER_SIZE], payload, payload_length);
    }

    s_tx_length = (uint16_t)(USB_PROTOCOL_HEADER_SIZE + payload_length);
    s_tx_offset = 0U;
    s_tx_active = true;
    return true;
}

static UsbStatus i2c_usb_status(I2C_Status_t status)
{
    switch (status) {
    case I2C_OK:               return USB_STATUS_OK;
    case I2C_TIMEOUT:          return USB_STATUS_I2C_TIMEOUT;
    case I2C_NACK:             return USB_STATUS_I2C_NACK;
    case I2C_BUS_ERROR:
    case I2C_ARBITRATION_LOST: return USB_STATUS_I2C_BUS_ERROR;
    case I2C_BUSY:             return USB_STATUS_BUSY;
    case I2C_ERROR:
    default:                   return USB_STATUS_INVALID_ARGUMENT;
    }
}

static UsbStatus mctp_usb_status(MctpStatus status)
{
    switch (status) {
    case MCTP_OK:                     return USB_STATUS_OK;
    case MCTP_ERROR_INVALID_ARGUMENT: return USB_STATUS_INVALID_ARGUMENT;
    case MCTP_ERROR_TIMEOUT:          return USB_STATUS_MCTP_TIMEOUT;
    case MCTP_ERROR_NACK:             return USB_STATUS_I2C_NACK;
    case MCTP_ERROR_PEC:              return USB_STATUS_MCTP_PEC_ERROR;
    case MCTP_ERROR_PACKET_FORMAT:
    case MCTP_ERROR_PACKET_TOO_LARGE:
    case MCTP_ERROR_TRANSPORT:
    case MCTP_ERROR_NOT_SUPPORTED:
    default:                          return USB_STATUS_MCTP_ERROR;
    }
}

static void dispatch_request(void)
{
    static const uint8_t pong[] = { 'P', 'O', 'N', 'G' };
    static const uint8_t device_info[] =
        "STM32F411 SMBus Adapter;protocol=1;max_payload=512;usb=full-speed;adapter=i2c,smbus,mctp";
    const uint8_t version = s_rx_message[USB_HEADER_VERSION_OFFSET];
    const uint8_t type = s_rx_message[USB_HEADER_TYPE_OFFSET];
    const uint16_t opcode = load_le16(&s_rx_message[USB_HEADER_OPCODE_OFFSET]);
    const uint32_t sequence = load_le32(&s_rx_message[USB_HEADER_SEQUENCE_OFFSET]);
    const uint16_t payload_length =
        load_le16(&s_rx_message[USB_HEADER_PAYLOAD_LENGTH_OFFSET]);
    const uint8_t *payload = &s_rx_message[USB_PROTOCOL_HEADER_SIZE];

    if (version != USB_PROTOCOL_VERSION) {
        (void)begin_response(opcode, sequence, USB_STATUS_BAD_VERSION, NULL, 0U);
        return;
    }
    if (type != USB_MESSAGE_REQUEST) {
        (void)begin_response(opcode, sequence, USB_STATUS_BAD_MESSAGE_TYPE,
                             NULL, 0U);
        return;
    }

    switch (opcode) {
    case USB_OPCODE_PING:
        if (payload_length != 0U) {
            (void)begin_response(opcode, sequence, USB_STATUS_BAD_LENGTH,
                                 NULL, 0U);
        } else {
            (void)begin_response(opcode, sequence, USB_STATUS_OK,
                                 pong, sizeof(pong));
        }
        break;

    case USB_OPCODE_GET_INFO:
        if (payload_length != 0U) {
            (void)begin_response(opcode, sequence, USB_STATUS_BAD_LENGTH,
                                 NULL, 0U);
        } else {
            (void)begin_response(opcode, sequence, USB_STATUS_OK,
                                 device_info,
                                 (uint16_t)(sizeof(device_info) - 1U));
        }
        break;

    case USB_OPCODE_ECHO:
        (void)begin_response(opcode, sequence, USB_STATUS_OK,
                             payload, payload_length);
        break;

    case USB_OPCODE_I2C_DETECT:
        if (payload_length != 0U) {
            (void)begin_response(opcode, sequence, USB_STATUS_BAD_LENGTH,
                                 NULL, 0U);
        } else {
            uint8_t device_count;
            I2C_Status_t status = adapter_i2c_detect(
                s_adapter_response, &device_count, NULL, NULL, NULL);
            (void)device_count;
            (void)begin_response(
                opcode, sequence, i2c_usb_status(status),
                (status == I2C_OK) ? s_adapter_response : NULL,
                (status == I2C_OK) ? USB_I2C_ADDRESS_BITMAP_SIZE : 0U);
        }
        break;

    case USB_OPCODE_I2C_BITRATE:
        if (payload_length != sizeof(uint32_t)) {
            (void)begin_response(opcode, sequence, USB_STATUS_BAD_LENGTH,
                                 NULL, 0U);
        } else {
            I2C_Status_t status = adapter_i2c_bitrate(load_le32(payload));
            if (status == I2C_OK) {
                store_le32(s_adapter_response, adapter_i2c_get_bitrate());
            }
            (void)begin_response(opcode, sequence, i2c_usb_status(status),
                                 (status == I2C_OK) ? s_adapter_response : NULL,
                                 (status == I2C_OK) ? sizeof(uint32_t) : 0U);
        }
        break;

    case USB_OPCODE_I2C_SLAVE_ENABLE:
        if (payload_length != USB_I2C_SLAVE_CONFIG_SIZE) {
            (void)begin_response(opcode, sequence, USB_STATUS_BAD_LENGTH,
                                 NULL, 0U);
        } else if (payload[0] > 0x7FU) {
            (void)begin_response(opcode, sequence,
                                 USB_STATUS_INVALID_ARGUMENT, NULL, 0U);
        } else {
            uint16_t applied_tx;
            uint16_t applied_rx;
            I2C_Status_t status = adapter_i2c_slave_enable(
                payload[0], load_le16(&payload[1]), load_le16(&payload[3]),
                &applied_tx, &applied_rx);
            if (status == I2C_OK) {
                s_adapter_response[0] = payload[0];
                store_le16(&s_adapter_response[1], applied_tx);
                store_le16(&s_adapter_response[3], applied_rx);
            }
            (void)begin_response(
                opcode, sequence, i2c_usb_status(status),
                (status == I2C_OK) ? s_adapter_response : NULL,
                (status == I2C_OK) ? USB_I2C_SLAVE_CONFIG_SIZE : 0U);
        }
        break;

    case USB_OPCODE_ASYNC_POLL:
        if (payload_length != USB_ASYNC_POLL_REQUEST_SIZE) {
            (void)begin_response(opcode, sequence, USB_STATUS_BAD_LENGTH,
                                 NULL, 0U);
        } else {
            int32_t timeout_ms = (int32_t)load_le32(payload);
            uint8_t flags = adapter_async_poll();

            if ((timeout_ms < -1) ||
                (timeout_ms > USB_ASYNC_POLL_MAX_TIMEOUT_MS)) {
                (void)begin_response(opcode, sequence,
                                     USB_STATUS_INVALID_ARGUMENT, NULL, 0U);
            } else if ((flags != ADAPTER_ASYNC_NO_DATA) ||
                       (timeout_ms == 0)) {
                s_adapter_response[0] = flags;
                (void)begin_response(opcode, sequence, USB_STATUS_OK,
                                     s_adapter_response,
                                     USB_ASYNC_POLL_RESPONSE_SIZE);
            } else {
                s_async_poll_active = true;
                s_async_poll_infinite = (timeout_ms < 0);
                s_async_poll_sequence = sequence;
                s_async_poll_started = xTaskGetTickCount();
                if (!s_async_poll_infinite) {
                    s_async_poll_timeout_ticks =
                        pdMS_TO_TICKS((uint32_t)timeout_ms);
                    if (s_async_poll_timeout_ticks == 0U) {
                        s_async_poll_timeout_ticks = 1U;
                    }
                }
            }
        }
        break;

    case USB_OPCODE_I2C_WRITE:
        if (payload_length < USB_I2C_WRITE_PREFIX_SIZE) {
            (void)begin_response(opcode, sequence, USB_STATUS_BAD_LENGTH,
                                 NULL, 0U);
        } else {
            uint16_t flags = load_le16(&payload[1]);
            uint16_t data_length = load_le16(&payload[3]);

            if ((payload[0] > 0x7FU) || (flags != USB_I2C_NO_FLAGS) ||
                (data_length > USB_I2C_WRITE_MAX_DATA_LENGTH) ||
                (payload_length !=
                 (uint16_t)(USB_I2C_WRITE_PREFIX_SIZE + data_length))) {
                (void)begin_response(opcode, sequence,
                                     USB_STATUS_INVALID_ARGUMENT, NULL, 0U);
            } else {
                I2C_Status_t status = adapter_i2c_write(
                    payload[0], flags, &payload[USB_I2C_WRITE_PREFIX_SIZE],
                    data_length);
                if (status == I2C_OK) {
                    store_le16(s_adapter_response, data_length);
                }
                (void)begin_response(
                    opcode, sequence, i2c_usb_status(status),
                    (status == I2C_OK) ? s_adapter_response : NULL,
                    (status == I2C_OK) ? USB_I2C_WRITE_RESPONSE_SIZE : 0U);
            }
        }
        break;

    case USB_OPCODE_I2C_WRITE_EXT:
        if (payload_length < USB_I2C_WRITE_PREFIX_SIZE) {
            (void)begin_response(opcode, sequence, USB_STATUS_BAD_LENGTH,
                                 NULL, 0U);
        } else {
            uint16_t flags = load_le16(&payload[1]);
            uint16_t data_length = load_le16(&payload[3]);

            if ((payload[0] > 0x7FU) || (flags != USB_I2C_NO_FLAGS) ||
                (data_length > USB_I2C_WRITE_MAX_DATA_LENGTH) ||
                (payload_length !=
                 (uint16_t)(USB_I2C_WRITE_PREFIX_SIZE + data_length))) {
                (void)begin_response(opcode, sequence,
                                     USB_STATUS_INVALID_ARGUMENT, NULL, 0U);
            } else {
                uint16_t num_written;
                I2C_ExtendedStatus_t status = adapter_i2c_write_ext(
                    payload[0], flags, &payload[USB_I2C_WRITE_PREFIX_SIZE],
                    data_length, &num_written);

                s_adapter_response[0] = (uint8_t)status;
                store_le16(&s_adapter_response[1], num_written);
                (void)begin_response(opcode, sequence, USB_STATUS_OK,
                                     s_adapter_response,
                                     USB_I2C_WRITE_EXT_RESPONSE_SIZE);
            }
        }
        break;

    case USB_OPCODE_I2C_READ_EXT:
        if (payload_length != USB_I2C_READ_EXT_REQUEST_SIZE) {
            (void)begin_response(opcode, sequence, USB_STATUS_BAD_LENGTH,
                                 NULL, 0U);
        } else {
            uint16_t flags = load_le16(&payload[1]);
            uint16_t requested_length = load_le16(&payload[3]);

            if ((payload[0] > 0x7FU) || (flags != USB_I2C_NO_FLAGS) ||
                (requested_length == 0U) ||
                (requested_length > USB_I2C_READ_MAX_DATA_LENGTH)) {
                (void)begin_response(opcode, sequence,
                                     USB_STATUS_INVALID_ARGUMENT, NULL, 0U);
            } else {
                uint16_t num_read;
                I2C_ExtendedStatus_t status = adapter_i2c_read_ext(
                    payload[0], flags,
                    &s_adapter_response[USB_I2C_READ_EXT_PREFIX_SIZE],
                    requested_length, &num_read);

                s_adapter_response[0] = (uint8_t)status;
                store_le16(&s_adapter_response[1], num_read);
                (void)begin_response(
                    opcode, sequence, USB_STATUS_OK, s_adapter_response,
                    (uint16_t)(USB_I2C_READ_EXT_PREFIX_SIZE + num_read));
            }
        }
        break;

    case USB_OPCODE_I2C_SLAVE_READ_EXT:
        if (payload_length != USB_I2C_SLAVE_READ_EXT_REQUEST_SIZE) {
            (void)begin_response(opcode, sequence, USB_STATUS_BAD_LENGTH,
                                 NULL, 0U);
        } else {
            uint16_t buffer_size = load_le16(payload);

            if ((buffer_size == 0U) ||
                (buffer_size > USB_I2C_SLAVE_READ_MAX_DATA_LENGTH)) {
                (void)begin_response(opcode, sequence,
                                     USB_STATUS_INVALID_ARGUMENT, NULL, 0U);
            } else {
                uint8_t address;
                uint16_t num_read;
                I2C_ExtendedStatus_t transaction_status;
                I2C_Status_t status = adapter_i2c_slave_read_ext(
                    buffer_size, &address,
                    &s_adapter_response[USB_I2C_SLAVE_READ_EXT_PREFIX_SIZE],
                    &num_read, &transaction_status);

                if (status == I2C_OK) {
                    s_adapter_response[0] = (uint8_t)transaction_status;
                    s_adapter_response[1] = address;
                    store_le16(&s_adapter_response[2], num_read);
                }
                (void)begin_response(
                    opcode, sequence, i2c_usb_status(status),
                    (status == I2C_OK) ? s_adapter_response : NULL,
                    (status == I2C_OK)
                        ? (uint16_t)(USB_I2C_SLAVE_READ_EXT_PREFIX_SIZE +
                                     num_read)
                        : 0U);
            }
        }
        break;

    case USB_OPCODE_I2C_SLAVE_WRITE:
        if (payload_length < USB_I2C_SLAVE_WRITE_PREFIX_SIZE) {
            (void)begin_response(opcode, sequence, USB_STATUS_BAD_LENGTH,
                                 NULL, 0U);
        } else {
            uint16_t data_length = load_le16(payload);

            if ((data_length == 0U) ||
                (data_length > USB_I2C_SLAVE_WRITE_MAX_DATA_LENGTH) ||
                (payload_length !=
                 (uint16_t)(USB_I2C_SLAVE_WRITE_PREFIX_SIZE + data_length))) {
                (void)begin_response(opcode, sequence,
                                     USB_STATUS_INVALID_ARGUMENT, NULL, 0U);
            } else {
                uint16_t accepted_length;
                I2C_Status_t status = adapter_i2c_slave_write(
                    &payload[USB_I2C_SLAVE_WRITE_PREFIX_SIZE], data_length,
                    &accepted_length);

                if (status == I2C_OK) {
                    store_le16(s_adapter_response, accepted_length);
                }
                (void)begin_response(
                    opcode, sequence, i2c_usb_status(status),
                    (status == I2C_OK) ? s_adapter_response : NULL,
                    (status == I2C_OK)
                        ? USB_I2C_SLAVE_WRITE_RESPONSE_SIZE : 0U);
            }
        }
        break;

    case USB_OPCODE_SMBUS_READ:
        if ((payload_length != 2U) || (payload[0] > 0x7FU) ||
            (payload[1] == 0U) ||
            (payload[1] > USB_SMBUS_MAX_DATA_LENGTH)) {
            (void)begin_response(opcode, sequence,
                                 USB_STATUS_INVALID_ARGUMENT, NULL, 0U);
        } else {
            I2C_Status_t status = adapter_smbus_read(payload[0],
                                                     s_adapter_response,
                                                     payload[1]);
            (void)begin_response(opcode, sequence, i2c_usb_status(status),
                                 (status == I2C_OK) ? s_adapter_response : NULL,
                                 (status == I2C_OK) ? payload[1] : 0U);
        }
        break;

    case USB_OPCODE_SMBUS_WRITE:
        if ((payload_length < 4U) || (payload[0] > 0x7FU) ||
            (payload[2] == 0U) ||
            (payload[2] > USB_SMBUS_MAX_DATA_LENGTH) ||
            (payload_length != (uint16_t)(3U + payload[2]))) {
            (void)begin_response(opcode, sequence,
                                 USB_STATUS_INVALID_ARGUMENT, NULL, 0U);
        } else {
            I2C_Status_t status = adapter_smbus_write(payload[0], payload[1],
                                                      &payload[3], payload[2]);
            (void)begin_response(opcode, sequence, i2c_usb_status(status),
                                 NULL, 0U);
        }
        break;

    case USB_OPCODE_MCTP_SEND_RECEIVE:
        if ((payload_length < (USB_MCTP_REQUEST_PREFIX_SIZE +
                               MCTP_HEADER_SIZE)) ||
            (payload[0] > 0x7FU) ||
            ((payload_length - USB_MCTP_REQUEST_PREFIX_SIZE) >
             MCTP_PACKET_MAX_SIZE)) {
            (void)begin_response(opcode, sequence,
                                 USB_STATUS_INVALID_ARGUMENT, NULL, 0U);
        } else {
            MctpPacket response;
            MctpStatus status = adapter_mctp_send_receive(
                payload[0], &payload[USB_MCTP_REQUEST_PREFIX_SIZE],
                (uint16_t)(payload_length - USB_MCTP_REQUEST_PREFIX_SIZE),
                load_le32(&payload[1]), &response);
            if (status == MCTP_OK) {
                s_adapter_response[0] = response.source_smbus_address;
                memcpy(&s_adapter_response[USB_MCTP_RESPONSE_PREFIX_SIZE],
                       response.data, response.length);
            }
            (void)begin_response(
                opcode, sequence, mctp_usb_status(status),
                (status == MCTP_OK) ? s_adapter_response : NULL,
                (status == MCTP_OK)
                    ? (uint16_t)(USB_MCTP_RESPONSE_PREFIX_SIZE + response.length)
                    : 0U);
        }
        break;

    default:
        (void)begin_response(opcode, sequence, USB_STATUS_UNSUPPORTED_OPCODE,
                             NULL, 0U);
        break;
    }

    LOG_DEBUG("USB protocol: request opcode=0x%04X seq=%lu payload=%u",
              opcode, (unsigned long)sequence, (unsigned int)payload_length);
}

static void consume_rx_byte(uint8_t byte)
{
    if (s_rx_length >= sizeof(s_rx_message)) {
        reset_rx();
    }
    s_rx_message[s_rx_length++] = byte;

    /* Search for the four-byte magic without trusting transfer alignment. */
    while ((s_rx_length >= 4U) &&
           (load_le32(s_rx_message) != USB_PROTOCOL_MAGIC)) {
        memmove(s_rx_message, &s_rx_message[1], --s_rx_length);
    }

    if ((s_rx_length == USB_PROTOCOL_HEADER_SIZE) &&
        (s_rx_expected_length == USB_PROTOCOL_HEADER_SIZE)) {
        const uint16_t payload_length =
            load_le16(&s_rx_message[USB_HEADER_PAYLOAD_LENGTH_OFFSET]);

        if (payload_length > USB_PROTOCOL_MAX_PAYLOAD) {
            const uint16_t opcode =
                load_le16(&s_rx_message[USB_HEADER_OPCODE_OFFSET]);
            const uint32_t sequence =
                load_le32(&s_rx_message[USB_HEADER_SEQUENCE_OFFSET]);
            (void)begin_response(opcode, sequence, USB_STATUS_BAD_LENGTH,
                                 NULL, 0U);
            reset_rx();
            return;
        }
        s_rx_expected_length =
            (uint16_t)(USB_PROTOCOL_HEADER_SIZE + payload_length);
    }

    if ((s_rx_length >= USB_PROTOCOL_HEADER_SIZE) &&
        (s_rx_length == s_rx_expected_length)) {
        dispatch_request();
        reset_rx();
    }
}

static void process_rx(void)
{
    uint8_t bytes[64];
    uint32_t count;

    while ((count = tud_vendor_read(bytes, sizeof(bytes))) > 0U) {
        for (uint32_t index = 0U; index < count; index++) {
            consume_rx_byte(bytes[index]);
        }
    }
}

static void process_tx(void)
{
    while (s_tx_active) {
        uint32_t available = tud_vendor_write_available();
        uint16_t remaining;
        uint32_t to_write;
        uint32_t written;

        if (available == 0U) {
            break;
        }
        remaining = (uint16_t)(s_tx_length - s_tx_offset);
        to_write = (remaining < available) ? remaining : available;
        written = tud_vendor_write(&s_tx_message[s_tx_offset], to_write);
        s_tx_offset = (uint16_t)(s_tx_offset + written);
        (void)tud_vendor_write_flush();

        if (s_tx_offset == s_tx_length) {
            s_tx_active = false;
        }
        if (written == 0U) {
            break;
        }
    }
}

static void process_async_poll(void)
{
    uint8_t flags;
    bool expired;

    if (!s_async_poll_active) {
        return;
    }
    flags = adapter_async_poll();
    expired = !s_async_poll_infinite &&
              ((TickType_t)(xTaskGetTickCount() - s_async_poll_started) >=
               s_async_poll_timeout_ticks);
    if ((flags == ADAPTER_ASYNC_NO_DATA) && !expired) {
        return;
    }

    s_adapter_response[0] = flags;
    if (begin_response(USB_OPCODE_ASYNC_POLL, s_async_poll_sequence,
                       USB_STATUS_OK, s_adapter_response,
                       USB_ASYNC_POLL_RESPONSE_SIZE)) {
        s_async_poll_active = false;
    }
}

void usb_protocol_init(void)
{
    usb_protocol_reset();
}

void usb_protocol_reset(void)
{
    reset_rx();
    s_tx_length = 0U;
    s_tx_offset = 0U;
    s_tx_active = false;
    s_async_poll_active = false;
}

void usb_protocol_task(void)
{
    if (!tud_vendor_mounted()) {
        return;
    }
    process_rx();
    process_async_poll();
    process_tx();
}

void tud_vendor_rx_cb(uint8_t interface_index,
                      const uint8_t *buffer,
                      uint32_t buffer_size)
{
    /* Buffered mode: TinyUSB passes NULL/0 and usb_protocol_task() drains
     * the class FIFO immediately after tud_task() returns. */
    (void)interface_index;
    (void)buffer;
    (void)buffer_size;
}
