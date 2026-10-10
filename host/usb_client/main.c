#include <libusb.h>

#include "adapter_discovery.h"
#include "usb_protocol.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define USB_TIMEOUT_MS     65000U
#define USB_PACKET_SIZE    64U

typedef struct {
    uint16_t opcode;
    uint32_t sequence;
    uint16_t status;
    uint16_t payload_length;
    uint8_t payload[USB_PROTOCOL_MAX_PAYLOAD];
} Response;

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

static int send_all(libusb_device_handle *device,
                    const uint8_t *message,
                    uint16_t message_length)
{
    uint16_t offset = 0U;

    while (offset < message_length) {
        int transferred = 0;
        int result = libusb_bulk_transfer(device, USB_PROTOCOL_EP_OUT,
                                          (unsigned char *)&message[offset],
                                          message_length - offset,
                                          &transferred, USB_TIMEOUT_MS);
        if (result != LIBUSB_SUCCESS) {
            fprintf(stderr, "bulk OUT failed: %s\n", libusb_error_name(result));
            return result;
        }
        if (transferred <= 0) {
            fprintf(stderr, "bulk OUT made no progress\n");
            return LIBUSB_ERROR_IO;
        }
        offset = (uint16_t)(offset + transferred);
    }
    return LIBUSB_SUCCESS;
}

static int receive_response(libusb_device_handle *device,
                            uint16_t expected_opcode,
                            uint32_t expected_sequence,
                            unsigned int timeout_ms,
                            Response *response)
{
    uint8_t message[USB_PROTOCOL_MAX_MESSAGE];
    uint16_t length = 0U;
    uint16_t expected_length = USB_PROTOCOL_HEADER_SIZE;

    while (length < expected_length) {
        uint8_t packet[USB_PACKET_SIZE];
        int transferred = 0;
        int result = libusb_bulk_transfer(device, USB_PROTOCOL_EP_IN,
                                          packet, sizeof(packet),
                                          &transferred, timeout_ms);
        if (result != LIBUSB_SUCCESS) {
            fprintf(stderr, "bulk IN failed: %s\n", libusb_error_name(result));
            return result;
        }
        if (transferred == 0) {
            continue; /* A legal zero-length packet after a full USB packet. */
        }
        if ((uint32_t)length + (uint32_t)transferred > sizeof(message)) {
            fprintf(stderr, "response exceeds protocol maximum\n");
            return LIBUSB_ERROR_OVERFLOW;
        }
        memcpy(&message[length], packet, (size_t)transferred);
        length = (uint16_t)(length + transferred);

        if (length >= USB_PROTOCOL_HEADER_SIZE) {
            uint16_t payload_length;

            if (load_le32(&message[USB_HEADER_MAGIC_OFFSET]) !=
                USB_PROTOCOL_MAGIC) {
                fprintf(stderr, "invalid response magic\n");
                return LIBUSB_ERROR_IO;
            }
            payload_length =
                load_le16(&message[USB_HEADER_PAYLOAD_LENGTH_OFFSET]);
            if (payload_length > USB_PROTOCOL_MAX_PAYLOAD) {
                fprintf(stderr, "invalid response payload length: %u\n",
                        payload_length);
                return LIBUSB_ERROR_OVERFLOW;
            }
            expected_length =
                (uint16_t)(USB_PROTOCOL_HEADER_SIZE + payload_length);
        }
    }

    if (length != expected_length) {
        fprintf(stderr, "response framing mismatch: got %u, expected %u\n",
                length, expected_length);
        return LIBUSB_ERROR_IO;
    }
    if ((message[USB_HEADER_VERSION_OFFSET] != USB_PROTOCOL_VERSION) ||
        (message[USB_HEADER_TYPE_OFFSET] != USB_MESSAGE_RESPONSE) ||
        (load_le16(&message[USB_HEADER_OPCODE_OFFSET]) != expected_opcode) ||
        (load_le32(&message[USB_HEADER_SEQUENCE_OFFSET]) != expected_sequence)) {
        fprintf(stderr, "response header does not match request\n");
        return LIBUSB_ERROR_IO;
    }

    response->opcode = expected_opcode;
    response->sequence = expected_sequence;
    response->status = load_le16(&message[USB_HEADER_STATUS_OFFSET]);
    response->payload_length =
        load_le16(&message[USB_HEADER_PAYLOAD_LENGTH_OFFSET]);
    memcpy(response->payload, &message[USB_PROTOCOL_HEADER_SIZE],
           response->payload_length);
    return LIBUSB_SUCCESS;
}

static int exchange_with_timeout(libusb_device_handle *device,
                                 uint16_t opcode,
                                 uint32_t sequence,
                                 const uint8_t *payload,
                                 uint16_t payload_length,
                                 unsigned int receive_timeout_ms,
                                 Response *response)
{
    uint8_t request[USB_PROTOCOL_MAX_MESSAGE];
    int result;

    if (payload_length > USB_PROTOCOL_MAX_PAYLOAD) {
        return LIBUSB_ERROR_INVALID_PARAM;
    }
    store_le32(&request[USB_HEADER_MAGIC_OFFSET], USB_PROTOCOL_MAGIC);
    request[USB_HEADER_VERSION_OFFSET] = USB_PROTOCOL_VERSION;
    request[USB_HEADER_TYPE_OFFSET] = USB_MESSAGE_REQUEST;
    store_le16(&request[USB_HEADER_OPCODE_OFFSET], opcode);
    store_le32(&request[USB_HEADER_SEQUENCE_OFFSET], sequence);
    store_le16(&request[USB_HEADER_PAYLOAD_LENGTH_OFFSET], payload_length);
    store_le16(&request[USB_HEADER_STATUS_OFFSET], 0U);
    if (payload_length > 0U) {
        memcpy(&request[USB_PROTOCOL_HEADER_SIZE], payload, payload_length);
    }

    result = send_all(device, request,
                      (uint16_t)(USB_PROTOCOL_HEADER_SIZE + payload_length));
    if (result != LIBUSB_SUCCESS) {
        return result;
    }
    return receive_response(device, opcode, sequence, receive_timeout_ms,
                            response);
}

static int exchange(libusb_device_handle *device,
                    uint16_t opcode,
                    uint32_t sequence,
                    const uint8_t *payload,
                    uint16_t payload_length,
                    Response *response)
{
    return exchange_with_timeout(device, opcode, sequence, payload,
                                 payload_length, USB_TIMEOUT_MS, response);
}

static int expect_success(const Response *response)
{
    if (response->status != USB_STATUS_OK) {
        static const char *const names[] = {
            "OK", "bad version", "bad message type", "bad length",
            "unsupported opcode", "busy", "invalid argument", "I2C timeout",
            "I2C NACK", "I2C bus error", "MCTP error", "MCTP timeout",
            "MCTP PEC error"
        };
        const char *name = (response->status <
                            (sizeof(names) / sizeof(names[0])))
                               ? names[response->status] : "unknown";
        fprintf(stderr, "device returned status %u (%s)\n",
                response->status, name);
        return 1;
    }
    return 0;
}

static bool parse_u32(const char *text, uint32_t maximum, uint32_t *value)
{
    char *end;
    unsigned long parsed;

    if ((text == NULL) || (text[0] == '\0')) {
        return false;
    }
    parsed = strtoul(text, &end, 0);
    if ((*end != '\0') || (parsed > maximum)) {
        return false;
    }
    *value = (uint32_t)parsed;
    return true;
}

static bool parse_bytes(char **arguments, int count,
                        uint8_t *bytes, uint16_t capacity)
{
    if ((count < 0) || ((uint16_t)count > capacity)) {
        return false;
    }
    for (int index = 0; index < count; index++) {
        uint32_t value;
        if (!parse_u32(arguments[index], 0xFFU, &value)) {
            return false;
        }
        bytes[index] = (uint8_t)value;
    }
    return true;
}

static void print_bytes(const uint8_t *bytes, uint16_t length)
{
    for (uint16_t index = 0U; index < length; index++) {
        printf("%s%02X", (index == 0U) ? "" : " ", bytes[index]);
    }
    putchar('\n');
}

static int run_i2c_detect(libusb_device_handle *device, uint32_t sequence)
{
    Response response;
    int result = exchange(device, USB_OPCODE_I2C_DETECT, sequence,
                          NULL, 0U, &response);
    unsigned int count = 0U;

    if ((result != LIBUSB_SUCCESS) || expect_success(&response)) {
        return 1;
    }
    if (response.payload_length != USB_I2C_ADDRESS_BITMAP_SIZE) {
        fprintf(stderr, "invalid I2C_DETECT response length\n");
        return 1;
    }
    for (uint8_t address = 0x03U; address <= 0x77U; address++) {
        if ((response.payload[address >> 3U] &
             (uint8_t)(1U << (address & 7U))) != 0U) {
            printf("0x%02X\n", address);
            count++;
        }
    }
    printf("%u device(s) found\n", count);
    return 0;
}

static int run_i2c_bitrate(libusb_device_handle *device,
                           uint32_t sequence,
                           uint32_t bitrate_hz)
{
    uint8_t request[sizeof(uint32_t)];
    Response response;
    int result;

    store_le32(request, bitrate_hz);
    result = exchange(device, USB_OPCODE_I2C_BITRATE, sequence,
                      request, sizeof(request), &response);
    if ((result != LIBUSB_SUCCESS) || expect_success(&response)) {
        return 1;
    }
    if (response.payload_length != sizeof(uint32_t)) {
        fprintf(stderr, "invalid I2C_BITRATE response length\n");
        return 1;
    }
    printf("I2C bitrate: %u Hz\n", load_le32(response.payload));
    return 0;
}

static int run_i2c_slave_enable(libusb_device_handle *handle,
                                uint32_t sequence,
                                uint8_t slave_address,
                                uint16_t max_tx_bytes,
                                uint16_t max_rx_bytes)
{
    uint8_t request[USB_I2C_SLAVE_CONFIG_SIZE];
    Response response;
    int result;

    request[0] = slave_address;
    store_le16(&request[1], max_tx_bytes);
    store_le16(&request[3], max_rx_bytes);
    result = exchange(handle, USB_OPCODE_I2C_SLAVE_ENABLE, sequence,
                      request, sizeof(request), &response);
    if ((result != LIBUSB_SUCCESS) || expect_success(&response)) {
        return 1;
    }
    if ((response.payload_length != USB_I2C_SLAVE_CONFIG_SIZE) ||
        (response.payload[0] != slave_address)) {
        fprintf(stderr, "invalid I2C_SLAVE_ENABLE response\n");
        return 1;
    }
    printf("I2C slave enabled: address=0x%02X max_tx=%u max_rx=%u\n",
           response.payload[0], load_le16(&response.payload[1]),
           load_le16(&response.payload[3]));
    return 0;
}

static int run_async_poll(libusb_device_handle *handle,
                          uint32_t sequence,
                          int32_t timeout_ms)
{
    uint8_t request[USB_ASYNC_POLL_REQUEST_SIZE];
    Response response;
    unsigned int receive_timeout_ms = (timeout_ms < 0)
                                          ? 0U
                                          : (unsigned int)timeout_ms + 5000U;
    int result;

    store_le32(request, (uint32_t)timeout_ms);
    result = exchange_with_timeout(handle, USB_OPCODE_ASYNC_POLL, sequence,
                                   request, sizeof(request),
                                   receive_timeout_ms, &response);
    if ((result != LIBUSB_SUCCESS) || expect_success(&response)) {
        return 1;
    }
    if (response.payload_length != USB_ASYNC_POLL_RESPONSE_SIZE) {
        fprintf(stderr, "invalid ASYNC_POLL response length\n");
        return 1;
    }

    printf("async flags: 0x%02X", response.payload[0]);
    if (response.payload[0] == USB_ASYNC_NO_DATA) {
        printf(" (no data)");
    } else {
        if ((response.payload[0] & USB_ASYNC_I2C_READ) != 0U) {
            printf(" I2C_READ");
        }
        if ((response.payload[0] & USB_ASYNC_I2C_WRITE) != 0U) {
            printf(" I2C_WRITE");
        }
    }
    putchar('\n');
    return 0;
}

static const char *i2c_extended_status_name(uint8_t status)
{
    static const char *const names[] = {
        "OK", "BUS_ERROR", "SLA_ACK", "SLA_NACK", "DATA_NACK",
        "ARB_LOST", "BUS_LOCKED", "LAST_DATA_ACK"
    };

    return (status < (sizeof(names) / sizeof(names[0])))
               ? names[status] : "UNKNOWN";
}

static int run_i2c_write_ext(libusb_device_handle *handle,
                             uint32_t sequence,
                             uint8_t address,
                             uint16_t flags,
                             const uint8_t *data,
                             uint16_t length)
{
    uint8_t request[USB_PROTOCOL_MAX_PAYLOAD];
    Response response;
    uint8_t status;
    uint16_t num_written;
    int result;

    request[0] = address;
    store_le16(&request[1], flags);
    store_le16(&request[3], length);
    if (length > 0U) {
        memcpy(&request[USB_I2C_WRITE_PREFIX_SIZE], data, length);
    }
    result = exchange(handle, USB_OPCODE_I2C_WRITE_EXT, sequence, request,
                      (uint16_t)(USB_I2C_WRITE_PREFIX_SIZE + length),
                      &response);
    if ((result != LIBUSB_SUCCESS) || expect_success(&response)) {
        return 1;
    }
    if (response.payload_length != USB_I2C_WRITE_EXT_RESPONSE_SIZE) {
        fprintf(stderr, "invalid I2C_WRITE_EXT response\n");
        return 1;
    }
    status = response.payload[0];
    num_written = load_le16(&response.payload[1]);
    printf("I2C write ext: status=%u (%s) num_written=%u requested=%u\n",
           status, i2c_extended_status_name(status), num_written, length);
    return (status == USB_I2C_STATUS_OK) ? 0 : 1;
}

static int run_i2c_read_ext(libusb_device_handle *handle,
                            uint32_t sequence,
                            uint8_t address,
                            uint16_t flags,
                            uint16_t length)
{
    uint8_t request[USB_I2C_READ_EXT_REQUEST_SIZE];
    Response response;
    uint8_t status;
    uint16_t num_read;
    int result;

    request[0] = address;
    store_le16(&request[1], flags);
    store_le16(&request[3], length);
    result = exchange(handle, USB_OPCODE_I2C_READ_EXT, sequence, request,
                      sizeof(request), &response);
    if ((result != LIBUSB_SUCCESS) || expect_success(&response)) {
        return 1;
    }
    if (response.payload_length < USB_I2C_READ_EXT_PREFIX_SIZE) {
        fprintf(stderr, "invalid I2C_READ_EXT response\n");
        return 1;
    }
    status = response.payload[0];
    num_read = load_le16(&response.payload[1]);
    if ((num_read > length) ||
        (response.payload_length !=
         (uint16_t)(USB_I2C_READ_EXT_PREFIX_SIZE + num_read))) {
        fprintf(stderr, "invalid I2C_READ_EXT byte count\n");
        return 1;
    }
    printf("I2C read ext: status=%u (%s) num_read=%u requested=%u\n",
           status, i2c_extended_status_name(status), num_read, length);
    if (num_read > 0U) {
        print_bytes(&response.payload[USB_I2C_READ_EXT_PREFIX_SIZE], num_read);
    }
    return (status == USB_I2C_STATUS_OK) ? 0 : 1;
}

static int run_i2c_slave_read_ext(libusb_device_handle *handle,
                                  uint32_t sequence,
                                  uint16_t buffer_size)
{
    uint8_t request[USB_I2C_SLAVE_READ_EXT_REQUEST_SIZE];
    Response response;
    uint8_t status;
    uint8_t address;
    uint16_t num_read;
    int result;

    store_le16(request, buffer_size);
    result = exchange(handle, USB_OPCODE_I2C_SLAVE_READ_EXT, sequence,
                      request, sizeof(request), &response);
    if ((result != LIBUSB_SUCCESS) || expect_success(&response)) {
        return 1;
    }
    if (response.payload_length < USB_I2C_SLAVE_READ_EXT_PREFIX_SIZE) {
        fprintf(stderr, "invalid I2C_SLAVE_READ_EXT response\n");
        return 1;
    }
    status = response.payload[0];
    address = response.payload[1];
    num_read = load_le16(&response.payload[2]);
    if ((address > 0x7FU) || (num_read > buffer_size) ||
        (response.payload_length !=
         (uint16_t)(USB_I2C_SLAVE_READ_EXT_PREFIX_SIZE + num_read))) {
        fprintf(stderr, "invalid I2C_SLAVE_READ_EXT response fields\n");
        return 1;
    }
    printf("I2C slave read ext: status=%u (%s) address=0x%02X "
           "num_read=%u buffer_size=%u\n",
           status, i2c_extended_status_name(status), address, num_read,
           buffer_size);
    if (num_read > 0U) {
        print_bytes(&response.payload[USB_I2C_SLAVE_READ_EXT_PREFIX_SIZE],
                    num_read);
    }
    return (status == USB_I2C_STATUS_OK) ? 0 : 1;
}

static int run_smbus_read(libusb_device_handle *device, uint32_t sequence,
                          uint8_t address, uint8_t length)
{
    uint8_t request[2] = { address, length };
    Response response;
    int result = exchange(device, USB_OPCODE_SMBUS_READ, sequence,
                          request, sizeof(request), &response);

    if ((result != LIBUSB_SUCCESS) || expect_success(&response)) {
        return 1;
    }
    if (response.payload_length != length) {
        fprintf(stderr, "SMBus read returned %u bytes, expected %u\n",
                response.payload_length, length);
        return 1;
    }
    print_bytes(response.payload, response.payload_length);
    return 0;
}

static int run_smbus_write(libusb_device_handle *device, uint32_t sequence,
                           uint8_t address, uint8_t command,
                           const uint8_t *data, uint8_t length)
{
    uint8_t request[3U + USB_SMBUS_MAX_DATA_LENGTH];
    Response response;
    int result;

    request[0] = address;
    request[1] = command;
    request[2] = length;
    memcpy(&request[3], data, length);
    result = exchange(device, USB_OPCODE_SMBUS_WRITE, sequence,
                      request, (uint16_t)(3U + length), &response);
    if ((result != LIBUSB_SUCCESS) || expect_success(&response)) {
        return 1;
    }
    if (response.payload_length != 0U) {
        fprintf(stderr, "SMBus write returned unexpected data\n");
        return 1;
    }
    puts("SMBus write: OK");
    return 0;
}

static int run_mctp_send_receive(libusb_device_handle *device,
                                 uint32_t sequence,
                                 uint8_t address,
                                 uint32_t timeout_ms,
                                 const uint8_t *packet,
                                 uint16_t packet_length)
{
    uint8_t request[USB_MCTP_REQUEST_PREFIX_SIZE + USB_MCTP_PACKET_MAX_SIZE];
    Response response;
    int result;

    request[0] = address;
    store_le32(&request[1], timeout_ms);
    memcpy(&request[USB_MCTP_REQUEST_PREFIX_SIZE], packet, packet_length);
    result = exchange(device, USB_OPCODE_MCTP_SEND_RECEIVE, sequence,
                      request,
                      (uint16_t)(USB_MCTP_REQUEST_PREFIX_SIZE + packet_length),
                      &response);
    if ((result != LIBUSB_SUCCESS) || expect_success(&response)) {
        return 1;
    }
    if (response.payload_length <
        (USB_MCTP_RESPONSE_PREFIX_SIZE + USB_MCTP_PACKET_MIN_SIZE)) {
        fprintf(stderr, "invalid MCTP response length\n");
        return 1;
    }
    printf("source SMBus address: 0x%02X\nMCTP packet: ",
           response.payload[0]);
    print_bytes(&response.payload[USB_MCTP_RESPONSE_PREFIX_SIZE],
                (uint16_t)(response.payload_length -
                           USB_MCTP_RESPONSE_PREFIX_SIZE));
    return 0;
}

static int run_ping(libusb_device_handle *device, uint32_t sequence)
{
    Response response;
    int result = exchange(device, USB_OPCODE_PING, sequence, NULL, 0U, &response);

    if ((result != LIBUSB_SUCCESS) || expect_success(&response)) {
        return 1;
    }
    if ((response.payload_length != 4U) ||
        (memcmp(response.payload, "PONG", 4U) != 0)) {
        fprintf(stderr, "invalid PING response\n");
        return 1;
    }
    printf("PONG (sequence %u)\n", sequence);
    return 0;
}

static int run_info(libusb_device_handle *device, uint32_t sequence)
{
    Response response;
    int result = exchange(device, USB_OPCODE_GET_INFO, sequence,
                          NULL, 0U, &response);

    if ((result != LIBUSB_SUCCESS) || expect_success(&response)) {
        return 1;
    }
    printf("%.*s\n", response.payload_length, (const char *)response.payload);
    return 0;
}

static int run_echo(libusb_device_handle *device,
                    uint32_t sequence,
                    const uint8_t *payload,
                    uint16_t payload_length,
                    bool quiet)
{
    Response response;
    int result = exchange(device, USB_OPCODE_ECHO, sequence,
                          payload, payload_length, &response);

    if ((result != LIBUSB_SUCCESS) || expect_success(&response)) {
        return 1;
    }
    if ((response.payload_length != payload_length) ||
        (memcmp(response.payload, payload, payload_length) != 0)) {
        fprintf(stderr, "ECHO data mismatch at length %u\n", payload_length);
        return 1;
    }
    if (!quiet) {
        printf("ECHO %u byte(s): %.*s\n", payload_length, payload_length,
               (const char *)response.payload);
    }
    return 0;
}

static int run_selftest(libusb_device_handle *device)
{
    static const uint16_t sizes[] = {
        0U, 1U, 2U, 63U, 64U, 65U, 127U, 128U,
        255U, 256U, 511U, 512U
    };
    uint8_t payload[USB_PROTOCOL_MAX_PAYLOAD];
    uint32_t sequence = 1U;

    if (run_ping(device, sequence++) || run_info(device, sequence++)) {
        return 1;
    }
    for (size_t test = 0U; test < sizeof(sizes) / sizeof(sizes[0]); test++) {
        for (uint16_t index = 0U; index < sizes[test]; index++) {
            payload[index] = (uint8_t)((index * 37U + sizes[test]) & 0xFFU);
        }
        if (run_echo(device, sequence++, payload, sizes[test], true)) {
            return 1;
        }
        printf("ECHO length %3u: PASS\n", sizes[test]);
    }
    puts("Self-test: PASS");
    return 0;
}

static int run_find_devices(void)
{
    int found = find_devices(0, NULL);
    u16 *devices;
    int reported;

    if (found <= 0) {
        puts("0 device(s) found");
        return 0;
    }
    devices = calloc((size_t)found, sizeof(*devices));
    if (devices == NULL) {
        fprintf(stderr, "unable to allocate device list\n");
        return 1;
    }
    reported = find_devices(found, devices);
    printf("%d device(s) found\n", reported);
    for (int index = 0; index < found && index < reported; index++) {
        printf("port %u%s\n",
               devices[index] & (u16)~ADAPTER_PORT_NOT_FREE,
               (devices[index] & ADAPTER_PORT_NOT_FREE) ? " (in use)" : "");
    }
    free(devices);
    return 0;
}

static int run_open_ext(const char *port_text)
{
    AardvarkExt extended;
    uint32_t port;
    Aardvark handle;

    if (!parse_u32(port_text, UINT16_MAX, &port)) {
        fprintf(stderr, "invalid adapter port: %s\n", port_text);
        return 2;
    }
    handle = open_ext((int)port, &extended);
    if (handle < 0) {
        fprintf(stderr, "open_ext failed: %d\n", handle);
        return 1;
    }
    printf("handle %d: software=%u.%02u firmware=%u.%02u "
           "hardware=%u.%02u features=0x%X\n",
           handle,
           extended.version.software >> 8U,
           extended.version.software & 0xFFU,
           extended.version.firmware >> 8U,
           extended.version.firmware & 0xFFU,
           extended.version.hardware >> 8U,
           extended.version.hardware & 0xFFU,
           extended.features);
    return (adapter_close(handle) == 1) ? 0 : 1;
}

static void print_usage(const char *program)
{
    fprintf(stderr,
            "Usage:\n"
            "  %s find_devices\n"
            "  %s open_ext <port>\n"
            "  %s ping | info | echo <text> | selftest\n"
            "  %s i2c_detect\n"
            "  %s i2c_bitrate <100000|400000>\n"
            "  %s i2c_slave_enable <address> <max_tx_bytes> <max_rx_bytes>\n"
            "  %s i2c_slave_write <byte> [byte ...]\n"
            "  %s async_poll <timeout_ms|-1>\n"
            "  %s i2c_write <address> <I2C_NO_FLAGS|0> [byte ...]\n"
            "  %s i2c_write_ext <address> <I2C_NO_FLAGS|0> [byte ...]\n"
            "  %s i2c_read <address> <I2C_NO_FLAGS|0> <length>\n"
            "  %s i2c_read_ext <address> <I2C_NO_FLAGS|0> <length>\n"
            "  %s i2c_slave_read_ext <buffer_size 1..258>\n"
            "  %s smbus_read <address> <length 1..32>\n"
            "  %s smbus_write <address> <command> <byte> [byte ...]\n"
            "  %s mctp_send_receive <address> <timeout_ms> "
            "<MCTP byte> [byte ...]\n",
            program, program, program, program, program, program, program,
            program, program, program, program, program, program, program,
            program, program);
}

int main(int argc, char **argv)
{
    libusb_device_handle *device = NULL;
    Aardvark aardvark = 0;
    int exit_status = 1;

    if (argc < 2) {
        print_usage(argv[0]);
        return 2;
    }

    if ((strcmp(argv[1], "find_devices") == 0) && (argc == 2)) {
        return run_find_devices();
    }
    if ((strcmp(argv[1], "open_ext") == 0) && (argc == 3)) {
        return run_open_ext(argv[2]);
    }

    {
        int found = find_devices(0, NULL);
        int capacity = found;
        u16 *devices = NULL;
        u16 selected_port = 0U;
        bool selected = false;

        if (found > 0) {
            devices = calloc((size_t)found, sizeof(*devices));
        }
        if ((found <= 0) || (devices == NULL)) {
            fprintf(stderr, "no SMBus adapters found\n");
            free(devices);
            goto cleanup;
        }
        found = find_devices(capacity, devices);
        for (int index = 0; (index < found) && (index < capacity); index++) {
            if ((devices[index] & ADAPTER_PORT_NOT_FREE) == 0U) {
                selected_port = devices[index];
                selected = true;
                break;
            }
        }
        free(devices);
        if (!selected) {
            fprintf(stderr, "all discovered SMBus adapters are in use\n");
            goto cleanup;
        }
        aardvark = open_ext(selected_port, NULL);
        if (aardvark < 0) {
            fprintf(stderr, "unable to open SMBus adapter port %u: %d\n",
                    selected_port, aardvark);
            goto cleanup;
        }
        device = adapter_libusb_handle(aardvark);
    }
    if (device == NULL) {
        fprintf(stderr, "opened adapter has no USB handle\n");
        goto cleanup;
    }

    if (strcmp(argv[1], "ping") == 0) {
        exit_status = run_ping(device, 1U);
    } else if (strcmp(argv[1], "info") == 0) {
        exit_status = run_info(device, 1U);
    } else if ((strcmp(argv[1], "echo") == 0) && (argc == 3)) {
        size_t length = strlen(argv[2]);
        if (length > USB_PROTOCOL_MAX_PAYLOAD) {
            fprintf(stderr, "echo text exceeds %u bytes\n",
                    USB_PROTOCOL_MAX_PAYLOAD);
        } else {
            exit_status = run_echo(device, 1U, (const uint8_t *)argv[2],
                                   (uint16_t)length, false);
        }
    } else if (strcmp(argv[1], "selftest") == 0) {
        exit_status = run_selftest(device);
    } else if ((strcmp(argv[1], "i2c_detect") == 0) && (argc == 2)) {
        exit_status = run_i2c_detect(device, 1U);
    } else if ((strcmp(argv[1], "i2c_bitrate") == 0) && (argc == 3)) {
        uint32_t bitrate_hz;
        if (!parse_u32(argv[2], USB_I2C_BITRATE_FAST_HZ, &bitrate_hz) ||
            ((bitrate_hz != USB_I2C_BITRATE_STANDARD_HZ) &&
             (bitrate_hz != USB_I2C_BITRATE_FAST_HZ))) {
            print_usage(argv[0]);
        } else {
            exit_status = run_i2c_bitrate(device, 1U, bitrate_hz);
        }
    } else if ((strcmp(argv[1], "i2c_slave_enable") == 0) && (argc == 5)) {
        uint32_t slave_address;
        uint32_t max_tx_bytes;
        uint32_t max_rx_bytes;
        if (!parse_u32(argv[2], 0x7FU, &slave_address) ||
            !parse_u32(argv[3], USB_I2C_SLAVE_DEFAULT_MAX_TX,
                       &max_tx_bytes) ||
            !parse_u32(argv[4], USB_I2C_SLAVE_DEFAULT_MAX_RX,
                       &max_rx_bytes)) {
            print_usage(argv[0]);
        } else {
            exit_status = run_i2c_slave_enable(
                device, 1U, (uint8_t)slave_address, (uint16_t)max_tx_bytes,
                (uint16_t)max_rx_bytes);
        }
    } else if ((strcmp(argv[1], "i2c_slave_write") == 0) && (argc >= 3) &&
               ((argc - 2) <= (int)USB_I2C_SLAVE_WRITE_MAX_DATA_LENGTH)) {
        uint8_t data[USB_I2C_SLAVE_WRITE_MAX_DATA_LENGTH];

        if (!parse_bytes(&argv[2], argc - 2, data, sizeof(data))) {
            print_usage(argv[0]);
        } else {
            int accepted = i2c_slave_write(
                aardvark, (u16)(argc - 2), data);

            if (accepted < 0) {
                fprintf(stderr, "i2c_slave_write failed: %d\n", accepted);
            } else {
                printf("I2C slave response: %d byte(s) accepted\n", accepted);
                exit_status = 0;
            }
        }
    } else if ((strcmp(argv[1], "async_poll") == 0) && (argc == 3)) {
        uint32_t timeout_value;
        int32_t timeout_ms;

        if (strcmp(argv[2], "-1") == 0) {
            timeout_ms = -1;
        } else if (parse_u32(argv[2], USB_ASYNC_POLL_MAX_TIMEOUT_MS,
                             &timeout_value)) {
            timeout_ms = (int32_t)timeout_value;
        } else {
            print_usage(argv[0]);
            goto command_done;
        }
        exit_status = run_async_poll(device, 1U, timeout_ms);
    } else if ((strcmp(argv[1], "i2c_write") == 0) && (argc >= 4) &&
               ((argc - 4) <= (int)USB_I2C_WRITE_MAX_DATA_LENGTH)) {
        uint32_t address;
        uint32_t numeric_flags;
        uint16_t flags;
        uint8_t data[USB_I2C_WRITE_MAX_DATA_LENGTH];

        if (strcmp(argv[3], "I2C_NO_FLAGS") == 0) {
            flags = USB_I2C_NO_FLAGS;
        } else if (parse_u32(argv[3], UINT16_MAX, &numeric_flags)) {
            flags = (uint16_t)numeric_flags;
        } else {
            print_usage(argv[0]);
            goto command_done;
        }
        if (!parse_u32(argv[2], 0x7FU, &address) ||
            (flags != USB_I2C_NO_FLAGS) ||
            !parse_bytes(&argv[4], argc - 4, data, sizeof(data))) {
            print_usage(argv[0]);
        } else {
            int written = i2c_write(
                aardvark, (u16)address, flags, (u16)(argc - 4), data);

            if (written < 0) {
                fprintf(stderr, "i2c_write failed: %d\n", written);
            } else {
                printf("I2C write: %d byte(s) transmitted to 0x%02X\n",
                       written, (unsigned int)address);
                exit_status = 0;
            }
        }
    } else if ((strcmp(argv[1], "i2c_write_ext") == 0) && (argc >= 4) &&
               ((argc - 4) <= (int)USB_I2C_WRITE_MAX_DATA_LENGTH)) {
        uint32_t address;
        uint32_t numeric_flags;
        uint16_t flags;
        uint8_t data[USB_I2C_WRITE_MAX_DATA_LENGTH];

        if (strcmp(argv[3], "I2C_NO_FLAGS") == 0) {
            flags = USB_I2C_NO_FLAGS;
        } else if (parse_u32(argv[3], UINT16_MAX, &numeric_flags)) {
            flags = (uint16_t)numeric_flags;
        } else {
            print_usage(argv[0]);
            goto command_done;
        }
        if (!parse_u32(argv[2], 0x7FU, &address) ||
            (flags != USB_I2C_NO_FLAGS) ||
            !parse_bytes(&argv[4], argc - 4, data, sizeof(data))) {
            print_usage(argv[0]);
        } else {
            exit_status = run_i2c_write_ext(
                device, 1U, (uint8_t)address, flags, data,
                (uint16_t)(argc - 4));
        }
    } else if ((strcmp(argv[1], "i2c_read") == 0) && (argc == 5)) {
        uint32_t address;
        uint32_t numeric_flags;
        uint32_t length;
        uint16_t flags;
        uint8_t data[USB_I2C_READ_MAX_DATA_LENGTH];

        if (strcmp(argv[3], "I2C_NO_FLAGS") == 0) {
            flags = I2C_NO_FLAGS;
        } else if (parse_u32(argv[3], UINT16_MAX, &numeric_flags)) {
            flags = (uint16_t)numeric_flags;
        } else {
            print_usage(argv[0]);
            goto command_done;
        }
        if (!parse_u32(argv[2], 0x7FU, &address) ||
            (flags != I2C_NO_FLAGS) ||
            !parse_u32(argv[4], USB_I2C_READ_MAX_DATA_LENGTH, &length) ||
            (length == 0U)) {
            print_usage(argv[0]);
        } else {
            int num_read = i2c_read(aardvark, (u16)address, flags,
                                    (u16)length, data);

            if (num_read < 0) {
                fprintf(stderr, "i2c_read failed: %d\n", num_read);
            } else {
                printf("I2C read: %d byte(s) received from 0x%02X\n",
                       num_read, (unsigned int)address);
                print_bytes(data, (u16)num_read);
                exit_status = 0;
            }
        }
    } else if ((strcmp(argv[1], "i2c_read_ext") == 0) && (argc == 5)) {
        uint32_t address;
        uint32_t numeric_flags;
        uint32_t length;
        uint16_t flags;

        if (strcmp(argv[3], "I2C_NO_FLAGS") == 0) {
            flags = USB_I2C_NO_FLAGS;
        } else if (parse_u32(argv[3], UINT16_MAX, &numeric_flags)) {
            flags = (uint16_t)numeric_flags;
        } else {
            print_usage(argv[0]);
            goto command_done;
        }
        if (!parse_u32(argv[2], 0x7FU, &address) ||
            (flags != USB_I2C_NO_FLAGS) ||
            !parse_u32(argv[4], USB_I2C_READ_MAX_DATA_LENGTH, &length) ||
            (length == 0U)) {
            print_usage(argv[0]);
        } else {
            exit_status = run_i2c_read_ext(
                device, 1U, (uint8_t)address, flags, (uint16_t)length);
        }
    } else if ((strcmp(argv[1], "i2c_slave_read_ext") == 0) &&
               (argc == 3)) {
        uint32_t buffer_size;

        if (!parse_u32(argv[2], USB_I2C_SLAVE_READ_MAX_DATA_LENGTH,
                       &buffer_size) || (buffer_size == 0U)) {
            print_usage(argv[0]);
        } else {
            exit_status = run_i2c_slave_read_ext(
                device, 1U, (uint16_t)buffer_size);
        }
    } else if ((strcmp(argv[1], "smbus_read") == 0) && (argc == 4)) {
        uint32_t address;
        uint32_t length;
        if (!parse_u32(argv[2], 0x7FU, &address) ||
            !parse_u32(argv[3], USB_SMBUS_MAX_DATA_LENGTH, &length) ||
            (length == 0U)) {
            print_usage(argv[0]);
        } else {
            exit_status = run_smbus_read(device, 1U, (uint8_t)address,
                                          (uint8_t)length);
        }
    } else if ((strcmp(argv[1], "smbus_write") == 0) &&
               (argc >= 5) &&
               ((argc - 4) <= (int)USB_SMBUS_MAX_DATA_LENGTH)) {
        uint32_t address;
        uint32_t command;
        uint8_t data[USB_SMBUS_MAX_DATA_LENGTH];
        if (!parse_u32(argv[2], 0x7FU, &address) ||
            !parse_u32(argv[3], 0xFFU, &command) ||
            !parse_bytes(&argv[4], argc - 4, data, sizeof(data))) {
            print_usage(argv[0]);
        } else {
            exit_status = run_smbus_write(device, 1U, (uint8_t)address,
                                           (uint8_t)command, data,
                                           (uint8_t)(argc - 4));
        }
    } else if ((strcmp(argv[1], "mctp_send_receive") == 0) &&
               (argc >= 8) &&
               ((argc - 4) <= (int)USB_MCTP_PACKET_MAX_SIZE)) {
        uint32_t address;
        uint32_t timeout_ms;
        uint8_t packet[USB_MCTP_PACKET_MAX_SIZE];
        if (!parse_u32(argv[2], 0x7FU, &address) ||
            !parse_u32(argv[3], 60000U, &timeout_ms) ||
            !parse_bytes(&argv[4], argc - 4, packet, sizeof(packet))) {
            print_usage(argv[0]);
        } else {
            exit_status = run_mctp_send_receive(
                device, 1U, (uint8_t)address, timeout_ms, packet,
                (uint16_t)(argc - 4));
        }
    } else {
        print_usage(argv[0]);
        exit_status = 2;
    }

command_done:

cleanup:
    if (aardvark > 0) {
        (void)adapter_close(aardvark);
    }
    return exit_status;
}
