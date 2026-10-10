#ifndef SMBUS_USB_PROTOCOL_H
#define SMBUS_USB_PROTOCOL_H

#include <stdint.h>

/* On-wire multi-byte fields are little-endian. Bulk-transfer boundaries are
 * not message boundaries; receivers must reconstruct header + payload. */
#define USB_PROTOCOL_MAGIC             0x41424D53UL /* Bytes: "SMBA" */
#define USB_PROTOCOL_VERSION           1U
#define USB_PROTOCOL_HEADER_SIZE       16U
#define USB_PROTOCOL_MAX_PAYLOAD       512U
#define USB_PROTOCOL_MAX_MESSAGE       (USB_PROTOCOL_HEADER_SIZE + USB_PROTOCOL_MAX_PAYLOAD)

#define USB_PROTOCOL_EP_OUT            0x01U
#define USB_PROTOCOL_EP_IN             0x81U
#define USB_PROTOCOL_INTERFACE         0U

typedef enum {
    USB_MESSAGE_REQUEST = 1,
    USB_MESSAGE_RESPONSE = 2
} UsbMessageType;

typedef enum {
    USB_OPCODE_PING = 0x0001,
    USB_OPCODE_GET_INFO = 0x0002,
    USB_OPCODE_ECHO = 0x0003,
    USB_OPCODE_I2C_DETECT = 0x0100,
    USB_OPCODE_SMBUS_READ = 0x0101,
    USB_OPCODE_SMBUS_WRITE = 0x0102,
    USB_OPCODE_I2C_BITRATE = 0x0103,
    USB_OPCODE_I2C_SLAVE_ENABLE = 0x0104,
    USB_OPCODE_ASYNC_POLL = 0x0105,
    USB_OPCODE_I2C_WRITE = 0x0106,
    USB_OPCODE_I2C_WRITE_EXT = 0x0107,
    USB_OPCODE_I2C_READ_EXT = 0x0108,
    USB_OPCODE_I2C_SLAVE_READ_EXT = 0x0109,
    USB_OPCODE_I2C_SLAVE_WRITE = 0x010A,
    USB_OPCODE_MCTP_SEND_RECEIVE = 0x0200
} UsbOpcode;

typedef enum {
    USB_STATUS_OK = 0,
    USB_STATUS_BAD_VERSION = 1,
    USB_STATUS_BAD_MESSAGE_TYPE = 2,
    USB_STATUS_BAD_LENGTH = 3,
    USB_STATUS_UNSUPPORTED_OPCODE = 4,
    USB_STATUS_BUSY = 5,
    USB_STATUS_INVALID_ARGUMENT = 6,
    USB_STATUS_I2C_TIMEOUT = 7,
    USB_STATUS_I2C_NACK = 8,
    USB_STATUS_I2C_BUS_ERROR = 9,
    USB_STATUS_MCTP_ERROR = 10,
    USB_STATUS_MCTP_TIMEOUT = 11,
    USB_STATUS_MCTP_PEC_ERROR = 12
} UsbStatus;

/* Adapter command payloads. All multi-byte fields are little-endian.
 *
 * I2C_DETECT request:  empty
 * I2C_DETECT response: 16-byte bitmap; bit N represents 7-bit address N
 *
 * I2C_BITRATE request:  bitrate_hz:u32 (100000 or 400000)
 * I2C_BITRATE response: applied_bitrate_hz:u32
 *
 * I2C_SLAVE_ENABLE request: slave_address:u8, max_tx_bytes:u16,
 *                           max_rx_bytes:u16
 * I2C_SLAVE_ENABLE response: slave_address:u8, applied_max_tx_bytes:u16,
 *                            applied_max_rx_bytes:u16
 * A zero maximum selects the firmware default maximum.
 *
 * ASYNC_POLL request: timeout_ms:i32 (-1 blocks indefinitely)
 * ASYNC_POLL response: pending_flags:u8
 * Pending flags are non-consuming and may be bitwise ORed.
 *
 * I2C_WRITE request: address:u8, flags:u16, data_length:u16,
 *                    data[data_length]
 * I2C_WRITE response: bytes_written:u16
 * I2C_WRITE_EXT uses the same request and responds with
 * status:u8, bytes_written:u16.
 * I2C_READ_EXT request: address:u8, flags:u16, requested_length:u16
 * I2C_READ_EXT response: status:u8, bytes_read:u16, data[bytes_read]
 * I2C_SLAVE_READ_EXT request: buffer_size:u16
 * I2C_SLAVE_READ_EXT response: status:u8, address:u8, bytes_read:u16,
 *                              data[bytes_read]
 * I2C_SLAVE_WRITE request: data_length:u16, data[data_length]
 * I2C_SLAVE_WRITE response: accepted_length:u16
 *
 * SMBUS_READ request:  address:u8, length:u8 (1..32)
 * SMBUS_READ response: raw bytes
 *
 * SMBUS_WRITE request: address:u8, command:u8, data_length:u8,
 *                      data[data_length] (1..32)
 * SMBUS_WRITE response: empty
 *
 * MCTP_SEND_RECEIVE request: destination_smbus_address:u8, timeout_ms:u32,
 *                            packet[4..256]
 * MCTP_SEND_RECEIVE response: source_smbus_address:u8, packet[4..256]
 * The packet starts at the MCTP transport header. The SMBus command, byte
 * count, source-slave-address byte and PEC are added/removed by firmware. */
#define USB_I2C_ADDRESS_BITMAP_SIZE       16U
#define USB_I2C_BITRATE_STANDARD_HZ    100000UL
#define USB_I2C_BITRATE_FAST_HZ        400000UL
#define USB_I2C_SLAVE_CONFIG_SIZE          5U
#define USB_I2C_SLAVE_DEFAULT_MAX_TX     256U
#define USB_I2C_SLAVE_DEFAULT_MAX_RX     258U
#define USB_ASYNC_POLL_REQUEST_SIZE         4U
#define USB_ASYNC_POLL_RESPONSE_SIZE        1U
#define USB_ASYNC_POLL_MAX_TIMEOUT_MS   60000L
#define USB_ASYNC_NO_DATA                0x00U
#define USB_ASYNC_I2C_READ               0x01U
#define USB_ASYNC_I2C_WRITE              0x02U
#define USB_I2C_NO_FLAGS                 0x0000U
#define USB_I2C_WRITE_PREFIX_SIZE            5U
#define USB_I2C_WRITE_RESPONSE_SIZE          2U
#define USB_I2C_WRITE_EXT_RESPONSE_SIZE      3U
#define USB_I2C_WRITE_MAX_DATA_LENGTH \
    (USB_PROTOCOL_MAX_PAYLOAD - USB_I2C_WRITE_PREFIX_SIZE)
#define USB_I2C_READ_EXT_REQUEST_SIZE         5U
#define USB_I2C_READ_EXT_PREFIX_SIZE          3U
#define USB_I2C_READ_MAX_DATA_LENGTH \
    (USB_PROTOCOL_MAX_PAYLOAD - USB_I2C_READ_EXT_PREFIX_SIZE)
#define USB_I2C_SLAVE_READ_EXT_REQUEST_SIZE   2U
#define USB_I2C_SLAVE_READ_EXT_PREFIX_SIZE    4U
#define USB_I2C_SLAVE_READ_MAX_DATA_LENGTH  258U
#define USB_I2C_SLAVE_WRITE_PREFIX_SIZE       2U
#define USB_I2C_SLAVE_WRITE_RESPONSE_SIZE     2U
#define USB_I2C_SLAVE_WRITE_MAX_DATA_LENGTH 256U
#define USB_I2C_STATUS_OK                 0U
#define USB_I2C_STATUS_BUS_ERROR          1U
#define USB_I2C_STATUS_SLA_ACK            2U
#define USB_I2C_STATUS_SLA_NACK           3U
#define USB_I2C_STATUS_DATA_NACK          4U
#define USB_I2C_STATUS_ARB_LOST           5U
#define USB_I2C_STATUS_BUS_LOCKED         6U
#define USB_I2C_STATUS_LAST_DATA_ACK      7U
#define USB_SMBUS_MAX_DATA_LENGTH         32U
#define USB_MCTP_REQUEST_PREFIX_SIZE       5U
#define USB_MCTP_RESPONSE_PREFIX_SIZE      1U
#define USB_MCTP_PACKET_MIN_SIZE            4U
#define USB_MCTP_PACKET_MAX_SIZE          256U

enum {
    USB_HEADER_MAGIC_OFFSET = 0,
    USB_HEADER_VERSION_OFFSET = 4,
    USB_HEADER_TYPE_OFFSET = 5,
    USB_HEADER_OPCODE_OFFSET = 6,
    USB_HEADER_SEQUENCE_OFFSET = 8,
    USB_HEADER_PAYLOAD_LENGTH_OFFSET = 12,
    USB_HEADER_STATUS_OFFSET = 14
};

#endif /* SMBUS_USB_PROTOCOL_H */
