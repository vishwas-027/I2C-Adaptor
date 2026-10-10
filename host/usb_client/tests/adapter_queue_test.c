#include "../adapter_discovery.h"
#include "usb_protocol.h"

#include <assert.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

struct libusb_context { int unused; };
struct libusb_device { int unused; };
struct libusb_device_handle { int unused; };

typedef struct {
    uint8_t data[USB_PROTOCOL_MAX_MESSAGE];
    uint16_t length;
    uint16_t offset;
} MockResponse;

static struct libusb_context s_context;
static struct libusb_device s_device;
static struct libusb_device_handle s_device_handle;
static MockResponse s_responses[4];
static unsigned int s_response_count;
static unsigned int s_response_index;
static int s_inject_stale_response;
static int s_delay_response;
static uint8_t s_last_write_first_byte;

static uint16_t load_le16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t load_le32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

static void store_le16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void store_le32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

static void append_response(uint16_t opcode, uint32_t sequence,
                            const uint8_t *payload, uint16_t payload_length)
{
    MockResponse *response = &s_responses[s_response_count++];

    assert(s_response_count <=
           (sizeof(s_responses) / sizeof(s_responses[0])));
    store_le32(&response->data[USB_HEADER_MAGIC_OFFSET], USB_PROTOCOL_MAGIC);
    response->data[USB_HEADER_VERSION_OFFSET] = USB_PROTOCOL_VERSION;
    response->data[USB_HEADER_TYPE_OFFSET] = USB_MESSAGE_RESPONSE;
    store_le16(&response->data[USB_HEADER_OPCODE_OFFSET], opcode);
    store_le32(&response->data[USB_HEADER_SEQUENCE_OFFSET], sequence);
    store_le16(&response->data[USB_HEADER_PAYLOAD_LENGTH_OFFSET],
               payload_length);
    store_le16(&response->data[USB_HEADER_STATUS_OFFSET], USB_STATUS_OK);
    if (payload_length > 0U) {
        memcpy(&response->data[USB_PROTOCOL_HEADER_SIZE], payload,
               payload_length);
    }
    response->length = (uint16_t)(USB_PROTOCOL_HEADER_SIZE + payload_length);
    response->offset = 0U;
}

static void prepare_response(const uint8_t *request, int request_length)
{
    uint8_t payload[USB_PROTOCOL_MAX_PAYLOAD];
    uint16_t opcode;
    uint32_t sequence;
    uint16_t payload_length = 0U;

    assert(request_length >= (int)USB_PROTOCOL_HEADER_SIZE);
    opcode = load_le16(&request[USB_HEADER_OPCODE_OFFSET]);
    sequence = load_le32(&request[USB_HEADER_SEQUENCE_OFFSET]);
    s_response_count = 0U;
    s_response_index = 0U;

    if (opcode == USB_OPCODE_I2C_WRITE) {
        uint16_t written = load_le16(
            &request[USB_PROTOCOL_HEADER_SIZE + 3U]);

        store_le16(payload, written);
        payload_length = USB_I2C_WRITE_RESPONSE_SIZE;
        if (written > 0U) {
            s_last_write_first_byte = request[
                USB_PROTOCOL_HEADER_SIZE + USB_I2C_WRITE_PREFIX_SIZE];
        }
    } else if (opcode == USB_OPCODE_I2C_READ_EXT) {
        uint16_t count = load_le16(
            &request[USB_PROTOCOL_HEADER_SIZE + 3U]);

        payload[0] = USB_I2C_STATUS_OK;
        store_le16(&payload[1], count);
        for (uint16_t index = 0U; index < count; index++) {
            payload[USB_I2C_READ_EXT_PREFIX_SIZE + index] =
                (uint8_t)(0xA0U + index);
        }
        payload_length = (uint16_t)(USB_I2C_READ_EXT_PREFIX_SIZE + count);
    } else if (opcode == USB_OPCODE_I2C_SLAVE_WRITE) {
        store_le16(payload,
                   load_le16(&request[USB_PROTOCOL_HEADER_SIZE]));
        payload_length = USB_I2C_SLAVE_WRITE_RESPONSE_SIZE;
    } else {
        assert(0 && "unexpected opcode");
    }

    if (s_inject_stale_response) {
        append_response(opcode, sequence - 1U, payload, payload_length);
        s_inject_stale_response = 0;
    }
    append_response(opcode, sequence, payload, payload_length);
}

int LIBUSB_CALL libusb_init(libusb_context **context)
{
    *context = &s_context;
    return LIBUSB_SUCCESS;
}

void LIBUSB_CALL libusb_exit(libusb_context *context)
{
    assert(context == &s_context);
}

ssize_t LIBUSB_CALL libusb_get_device_list(libusb_context *context,
                                            libusb_device ***list)
{
    static libusb_device *devices[] = { &s_device, NULL };

    assert(context == &s_context);
    *list = devices;
    return 1;
}

void LIBUSB_CALL libusb_free_device_list(libusb_device **list,
                                          int unref_devices)
{
    (void)list;
    (void)unref_devices;
}

int LIBUSB_CALL libusb_get_device_descriptor(
    libusb_device *device, struct libusb_device_descriptor *descriptor)
{
    assert(device == &s_device);
    memset(descriptor, 0, sizeof(*descriptor));
    descriptor->idVendor = 0xCAFEU;
    descriptor->idProduct = 0x4110U;
    descriptor->bcdDevice = 0x0100U;
    descriptor->iSerialNumber = 1U;
    return LIBUSB_SUCCESS;
}

uint8_t LIBUSB_CALL libusb_get_bus_number(libusb_device *device)
{
    assert(device == &s_device);
    return 1U;
}

int LIBUSB_CALL libusb_get_port_numbers(libusb_device *device,
                                        uint8_t *ports, int length)
{
    assert(device == &s_device);
    assert(length >= 1);
    ports[0] = 2U;
    return 1;
}

uint8_t LIBUSB_CALL libusb_get_device_address(libusb_device *device)
{
    assert(device == &s_device);
    return 3U;
}

int LIBUSB_CALL libusb_open(libusb_device *device,
                            libusb_device_handle **handle)
{
    assert(device == &s_device);
    *handle = &s_device_handle;
    return LIBUSB_SUCCESS;
}

void LIBUSB_CALL libusb_close(libusb_device_handle *handle)
{
    assert(handle == &s_device_handle);
}

libusb_device *LIBUSB_CALL libusb_get_device(libusb_device_handle *handle)
{
    assert(handle == &s_device_handle);
    return &s_device;
}

int LIBUSB_CALL libusb_claim_interface(libusb_device_handle *handle,
                                       int interface_number)
{
    assert(handle == &s_device_handle);
    assert(interface_number == 0);
    return LIBUSB_SUCCESS;
}

int LIBUSB_CALL libusb_release_interface(libusb_device_handle *handle,
                                         int interface_number)
{
    assert(handle == &s_device_handle);
    assert(interface_number == 0);
    return LIBUSB_SUCCESS;
}

int LIBUSB_CALL libusb_set_auto_detach_kernel_driver(
    libusb_device_handle *handle, int enable)
{
    assert(handle == &s_device_handle);
    assert(enable == 1);
    return LIBUSB_SUCCESS;
}

int LIBUSB_CALL libusb_get_string_descriptor_ascii(
    libusb_device_handle *handle, uint8_t descriptor_index,
    unsigned char *data, int length)
{
    static const char serial[] = "1234567890";

    assert(handle == &s_device_handle);
    assert(descriptor_index == 1U);
    assert(length >= (int)(sizeof(serial) - 1U));
    memcpy(data, serial, sizeof(serial) - 1U);
    return (int)(sizeof(serial) - 1U);
}

int LIBUSB_CALL libusb_bulk_transfer(libusb_device_handle *handle,
                                     unsigned char endpoint,
                                     unsigned char *data, int length,
                                     int *transferred,
                                     unsigned int timeout)
{
    assert(handle == &s_device_handle);
    assert(timeout > 0U);
    if (endpoint == USB_PROTOCOL_EP_OUT) {
        prepare_response(data, length);
        *transferred = length;
        return LIBUSB_SUCCESS;
    }
    assert(endpoint == USB_PROTOCOL_EP_IN);
    assert(s_response_index < s_response_count);
    if (s_delay_response) {
        const struct timespec delay = { .tv_sec = 0, .tv_nsec = 1000000L };
        (void)nanosleep(&delay, NULL);
    }
    {
        MockResponse *response = &s_responses[s_response_index];
        int remaining = (int)(response->length - response->offset);
        int amount = (length < remaining) ? length : remaining;

        memcpy(data, &response->data[response->offset], (size_t)amount);
        response->offset = (uint16_t)(response->offset + amount);
        if (response->offset == response->length) {
            s_response_index++;
        }
        *transferred = amount;
    }
    return LIBUSB_SUCCESS;
}

typedef struct {
    I2cQueue queue;
    int result;
} SubmitThread;

static void *submit_thread(void *argument)
{
    SubmitThread *submit = argument;

    submit->result = queue_submit(submit->queue);
    return NULL;
}

static void *write_thread(void *argument)
{
    Aardvark handle = *(Aardvark *)argument;

    for (unsigned int index = 0U; index < 100U; index++) {
        assert(i2c_write(handle, 0x50U, I2C_NO_FLAGS, 0U, NULL) == 0);
    }
    return NULL;
}

int main(void)
{
    u16 ports[1];
    u32 ids[1];
    Aardvark handle;
    I2cQueue queue;
    uint8_t write_data[] = { 0x10U, 0x20U };
    uint8_t read_data[8] = { 0U };
    pthread_t threads[4];

    assert(find_devices_ext(1, ports, 1, ids) == 1);
    assert(ports[0] == 0U);
    assert(ids[0] == 1234567890U);
    handle = open_ext(ports[0], NULL);
    assert(handle > 0);

    assert(i2c_write(handle, 0x50U, I2C_NO_FLAGS,
                     sizeof(write_data), write_data) ==
           (int)sizeof(write_data));
    assert(i2c_read(handle, 0x50U, I2C_NO_FLAGS,
                    sizeof(read_data), read_data) ==
           (int)sizeof(read_data));
    assert(read_data[0] == 0xA0U && read_data[7] == 0xA7U);

    queue = queue_create(handle);
    assert(queue > 0);
    assert(queue_i2c_write(queue, 0x50U, I2C_NO_FLAGS,
                           sizeof(write_data), write_data) == 0);
    write_data[0] = 0xEEU;
    memset(read_data, 0, sizeof(read_data));
    assert(queue_i2c_read(queue, 0x50U, I2C_NO_FLAGS,
                          sizeof(read_data), read_data) == 0);
    assert(queue_submit(queue) == 2);
    assert(s_last_write_first_byte == 0x10U);
    assert(read_data[0] == 0xA0U && read_data[7] == 0xA7U);
    assert(queue_clear(queue) == 0);

    for (unsigned int index = 0U; index < 255U; index++) {
        assert(queue_i2c_write(queue, 0x50U, I2C_NO_FLAGS,
                               0U, NULL) == 0);
    }
    assert(queue_i2c_write(queue, 0x50U, I2C_NO_FLAGS,
                           0U, NULL) == STATUS_QUEUE_FULL);
    assert(queue_clear(queue) == 0);

    for (unsigned int index = 0U; index < 255U; index++) {
        assert(queue_i2c_write(queue, 0x50U, I2C_NO_FLAGS,
                               0U, NULL) == 0);
    }
    {
        SubmitThread submit = { .queue = queue, .result = 0 };
        pthread_t worker;
        const struct timespec delay = { .tv_sec = 0, .tv_nsec = 10000000L };

        s_delay_response = 1;
        assert(pthread_create(&worker, NULL, submit_thread, &submit) == 0);
        (void)nanosleep(&delay, NULL);
        assert(adapter_close(handle) == STATUS_QUEUE_BUSY);
        assert(async_abort(queue) == 0);
        assert(pthread_join(worker, NULL) == 0);
        assert(submit.result == STATUS_ABORTED);
        s_delay_response = 0;
    }
    assert(queue_destroy(queue) == 0);

    s_inject_stale_response = 1;
    assert(i2c_write(handle, 0x50U, I2C_NO_FLAGS, 0U, NULL) == 0);

    for (unsigned int index = 0U; index < 4U; index++) {
        assert(pthread_create(&threads[index], NULL,
                              write_thread, &handle) == 0);
    }
    for (unsigned int index = 0U; index < 4U; index++) {
        assert(pthread_join(threads[index], NULL) == 0);
    }

    for (unsigned int index = 0U; index < 1000U; index++) {
        queue = queue_create(handle);
        assert(queue > 0);
        assert(queue_i2c_write(queue, 0x50U, I2C_NO_FLAGS,
                               sizeof(write_data), write_data) == 0);
        assert(queue_clear(queue) == 0);
        assert(queue_destroy(queue) == 0);
    }

    queue = queue_create(handle);
    assert(queue > 0);
    assert(queue_i2c_write(queue, 0x50U, I2C_NO_FLAGS,
                           sizeof(write_data), write_data) == 0);
    assert(adapter_close(handle) == 1);
    assert(queue_destroy(queue) == STATUS_INVALID_QUEUE);
    assert(queue_create(handle) == STATUS_INVALID_HANDLE);
    assert(adapter_close(handle) == STATUS_INVALID_HANDLE);
    puts("adapter queue tests passed");
    return 0;
}
