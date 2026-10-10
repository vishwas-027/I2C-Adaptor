#include "adapter_discovery.h"
#include "usb_protocol.h"

#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define ADAPTER_USB_VID       0xCAFEU
#define ADAPTER_USB_PID       0x4110U
#define ADAPTER_USB_INTERFACE 0U
#define ADAPTER_API_VERSION   0x0100U
#define ADAPTER_MIN_FW_VERSION 0x0100U
#define ADAPTER_HW_VERSION    0x0100U
#define ADAPTER_MAX_HANDLES   16U
#define ADAPTER_MAX_QUEUES    16U
#define ADAPTER_QUEUE_MAX_COMMANDS 255U
#define ADAPTER_USB_TIMEOUT_MS 65000U
#define ADAPTER_USB_PACKET_SIZE 64U
#define ADAPTER_USB_STALE_RESPONSE_LIMIT 16U

typedef struct {
    libusb_context *context;
    libusb_device_handle *device;
    u32 next_sequence;
    pthread_mutex_t io_mutex;
} AdapterHandle;

static AdapterHandle s_handles[ADAPTER_MAX_HANDLES];
static pthread_mutex_t s_registry_mutex = PTHREAD_MUTEX_INITIALIZER;

typedef enum {
    QUEUE_COMMAND_I2C_WRITE,
    QUEUE_COMMAND_I2C_READ
} QueueCommandType;

typedef enum {
    QUEUE_STATE_IDLE,
    QUEUE_STATE_SUBMITTING,
    QUEUE_STATE_ABORT_REQUESTED
} QueueState;

typedef struct {
    QueueCommandType type;
    u16 slave_address;
    AardvarkI2cFlags flags;
    u16 num_bytes;
    u08 *data;
} QueueCommand;

typedef struct {
    atomic_bool in_use;
    Aardvark aardvark;
    size_t command_count;
    QueueCommand commands[ADAPTER_QUEUE_MAX_COMMANDS];
    atomic_int state;
} QueueHandle;

static QueueHandle s_queues[ADAPTER_MAX_QUEUES];

/* The registry mutex must be held by every caller of this helper. */
static QueueHandle *queue_from_handle_locked(I2cQueue queue)
{
    if ((queue < 1) || ((unsigned int)queue > ADAPTER_MAX_QUEUES) ||
        !atomic_load(&s_queues[(size_t)queue - 1U].in_use)) {
        return NULL;
    }
    return &s_queues[(size_t)queue - 1U];
}

static void clear_queue_commands(QueueHandle *queue)
{
    for (size_t index = 0U; index < queue->command_count; index++) {
        if (queue->commands[index].type == QUEUE_COMMAND_I2C_WRITE) {
            free(queue->commands[index].data);
        }
        memset(&queue->commands[index], 0,
               sizeof(queue->commands[index]));
    }
    queue->command_count = 0U;
}

static void release_queues_for_adapter(Aardvark aardvark)
{
    for (size_t slot = 0U; slot < ADAPTER_MAX_QUEUES; slot++) {
        QueueHandle *queue = &s_queues[slot];

        if (atomic_load(&queue->in_use) &&
            (queue->aardvark == aardvark)) {
            clear_queue_commands(queue);
            queue->aardvark = 0;
            atomic_store(&queue->in_use, false);
            atomic_store(&queue->state, QUEUE_STATE_IDLE);
        }
    }
}

static bool adapter_queue_is_busy(Aardvark aardvark)
{
    for (size_t slot = 0U; slot < ADAPTER_MAX_QUEUES; slot++) {
        if (atomic_load(&s_queues[slot].in_use) &&
            (s_queues[slot].aardvark == aardvark) &&
            (atomic_load(&s_queues[slot].state) != QUEUE_STATE_IDLE)) {
            return true;
        }
    }
    return false;
}

static AdapterHandle *lock_adapter_handle(Aardvark aardvark)
{
    AdapterHandle *handle;

    (void)pthread_mutex_lock(&s_registry_mutex);
    if ((aardvark < 1) || ((unsigned int)aardvark > ADAPTER_MAX_HANDLES) ||
        (s_handles[(size_t)aardvark - 1U].device == NULL)) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return NULL;
    }
    handle = &s_handles[(size_t)aardvark - 1U];
    (void)pthread_mutex_lock(&handle->io_mutex);
    (void)pthread_mutex_unlock(&s_registry_mutex);
    return handle;
}

static void unlock_adapter_handle(AdapterHandle *handle)
{
    (void)pthread_mutex_unlock(&handle->io_mutex);
}

static int is_adapter(libusb_device *device)
{
    struct libusb_device_descriptor descriptor;

    if (libusb_get_device_descriptor(device, &descriptor) != LIBUSB_SUCCESS) {
        return 0;
    }
    return (descriptor.idVendor == ADAPTER_USB_VID) &&
           (descriptor.idProduct == ADAPTER_USB_PID);
}

static int port_is_free(libusb_device *device)
{
    libusb_device_handle *handle = NULL;
    int result = libusb_open(device, &handle);

    if (result != LIBUSB_SUCCESS) {
        return 0;
    }
    (void)libusb_set_auto_detach_kernel_driver(handle, 1);
    result = libusb_claim_interface(handle, ADAPTER_USB_INTERFACE);
    if (result == LIBUSB_SUCCESS) {
        (void)libusb_release_interface(handle, ADAPTER_USB_INTERFACE);
    }
    libusb_close(handle);
    return result == LIBUSB_SUCCESS;
}

static u32 device_unique_id(libusb_device *device)
{
    struct libusb_device_descriptor descriptor;
    libusb_device_handle *handle = NULL;
    unsigned char serial[11];
    char *end = NULL;
    unsigned long value;
    int length;

    if ((libusb_get_device_descriptor(device, &descriptor) != LIBUSB_SUCCESS) ||
        (descriptor.iSerialNumber == 0U) ||
        (libusb_open(device, &handle) != LIBUSB_SUCCESS)) {
        return 0U;
    }
    length = libusb_get_string_descriptor_ascii(handle,
                                                 descriptor.iSerialNumber,
                                                 serial,
                                                 sizeof(serial) - 1U);
    libusb_close(handle);
    if (length <= 0) {
        return 0U;
    }
    serial[length] = '\0';

    errno = 0;
    value = strtoul((const char *)serial, &end, 10);
    if ((errno == ERANGE) || (end != (char *)&serial[length]) ||
        (value == 0UL) || (value > UINT32_MAX)) {
        return 0U;
    }
    return (u32)value;
}

static int compare_devices(const void *left, const void *right)
{
    libusb_device *a = *(libusb_device *const *)left;
    libusb_device *b = *(libusb_device *const *)right;
    uint8_t a_ports[8];
    uint8_t b_ports[8];
    int a_depth;
    int b_depth;
    uint8_t a_bus = libusb_get_bus_number(a);
    uint8_t b_bus = libusb_get_bus_number(b);

    if (a_bus != b_bus) {
        return (a_bus < b_bus) ? -1 : 1;
    }
    a_depth = libusb_get_port_numbers(a, a_ports, sizeof(a_ports));
    b_depth = libusb_get_port_numbers(b, b_ports, sizeof(b_ports));
    if ((a_depth > 0) && (b_depth > 0)) {
        int common = (a_depth < b_depth) ? a_depth : b_depth;
        for (int index = 0; index < common; index++) {
            if (a_ports[index] != b_ports[index]) {
                return (a_ports[index] < b_ports[index]) ? -1 : 1;
            }
        }
        if (a_depth != b_depth) {
            return (a_depth < b_depth) ? -1 : 1;
        }
    }
    if (libusb_get_device_address(a) != libusb_get_device_address(b)) {
        return (libusb_get_device_address(a) < libusb_get_device_address(b))
                   ? -1 : 1;
    }
    return 0;
}

int find_devices_ext(int num_devices, u16 *devices,
                     int num_ids, u32 *unique_ids)
{
    libusb_context *context = NULL;
    libusb_device **list = NULL;
    libusb_device **matches = NULL;
    ssize_t list_length;
    int found = 0;
    int device_capacity = (num_devices > 0) ? num_devices : 0;
    int id_capacity = (num_ids > 0) ? num_ids : 0;
    int capacity;

    if ((devices != NULL) && (unique_ids != NULL)) {
        capacity = (device_capacity < id_capacity)
                       ? device_capacity : id_capacity;
    } else if (devices != NULL) {
        capacity = device_capacity;
    } else if (unique_ids != NULL) {
        capacity = id_capacity;
    } else {
        capacity = 0;
    }

    if (libusb_init(&context) != LIBUSB_SUCCESS) {
        return 0;
    }
    list_length = libusb_get_device_list(context, &list);
    if (list_length < 0) {
        libusb_exit(context);
        return 0;
    }

    matches = calloc((size_t)list_length, sizeof(*matches));
    if (matches == NULL) {
        libusb_free_device_list(list, 1);
        libusb_exit(context);
        return 0;
    }
    for (ssize_t index = 0; index < list_length; index++) {
        if (is_adapter(list[index])) {
            matches[found++] = list[index];
        }
    }
    qsort(matches, (size_t)found, sizeof(*matches), compare_devices);

    for (int index = 0; index < found; index++) {
        u16 port;

        port = (u16)index;
        if (index < capacity) {
            if (devices != NULL) {
                if (!port_is_free(matches[index])) {
                    port |= ADAPTER_PORT_NOT_FREE;
                }
                devices[index] = port;
            }
            if (unique_ids != NULL) {
                unique_ids[index] = device_unique_id(matches[index]);
            }
        }
    }

    free(matches);
    libusb_free_device_list(list, 1);
    libusb_exit(context);
    return found;
}

int find_devices(int num_devices, u16 *devices)
{
    return find_devices_ext(num_devices, devices, 0, NULL);
}

static void initialize_ext(AardvarkExt *extended_info)
{
    if (extended_info == NULL) {
        return;
    }
    memset(extended_info, 0, sizeof(*extended_info));
    extended_info->version.software = ADAPTER_API_VERSION;
    extended_info->version.fw_req_by_sw = ADAPTER_MIN_FW_VERSION;
    extended_info->version.api_req_by_sw = ADAPTER_API_VERSION;
}

static u16 usb_bcd_to_api_version(u16 bcd_version)
{
    unsigned int major_tens = (bcd_version >> 12U) & 0x0FU;
    unsigned int major_ones = (bcd_version >> 8U) & 0x0FU;
    unsigned int minor_tens = (bcd_version >> 4U) & 0x0FU;
    unsigned int minor_ones = bcd_version & 0x0FU;

    if ((major_tens > 9U) || (major_ones > 9U) ||
        (minor_tens > 9U) || (minor_ones > 9U)) {
        return 0U;
    }
    return (u16)((((major_tens * 10U) + major_ones) << 8U) |
                 ((minor_tens * 10U) + minor_ones));
}

static void close_handle_slot(size_t slot)
{
    if (s_handles[slot].device != NULL) {
        (void)libusb_release_interface(s_handles[slot].device,
                                       ADAPTER_USB_INTERFACE);
        libusb_close(s_handles[slot].device);
    }
    if (s_handles[slot].context != NULL) {
        libusb_exit(s_handles[slot].context);
    }
    s_handles[slot].device = NULL;
    s_handles[slot].context = NULL;
    s_handles[slot].next_sequence = 0U;
}

Aardvark open_ext(int port_number, AardvarkExt *extended_info)
{
    struct libusb_device_descriptor descriptor;
    libusb_context *context = NULL;
    libusb_device_handle *device;
    u16 firmware_version;
    size_t slot;
    int result;

    initialize_ext(extended_info);
    if ((port_number < 0) ||
        (((unsigned int)port_number & ADAPTER_PORT_NOT_FREE) != 0U) ||
        ((unsigned int)port_number > UINT16_MAX)) {
        return STATUS_UNABLE_TO_OPEN;
    }
    (void)pthread_mutex_lock(&s_registry_mutex);
    for (slot = 0U; slot < ADAPTER_MAX_HANDLES; slot++) {
        if (s_handles[slot].device == NULL) {
            break;
        }
    }
    if (slot == ADAPTER_MAX_HANDLES) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_UNABLE_TO_OPEN;
    }
    if (libusb_init(&context) != LIBUSB_SUCCESS) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_UNABLE_TO_OPEN;
    }
    device = adapter_open_port(context, (u16)port_number);
    if (device == NULL) {
        libusb_exit(context);
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_UNABLE_TO_OPEN;
    }

    if (libusb_get_device_descriptor(libusb_get_device(device),
                                     &descriptor) != LIBUSB_SUCCESS) {
        libusb_close(device);
        libusb_exit(context);
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_UNABLE_TO_OPEN;
    }
    firmware_version = usb_bcd_to_api_version(descriptor.bcdDevice);
    if (extended_info != NULL) {
        extended_info->version.firmware = firmware_version;
        extended_info->version.hardware = ADAPTER_HW_VERSION;
        extended_info->version.sw_req_by_fw = ADAPTER_API_VERSION;
        extended_info->features = ADAPTER_FEATURE_I2C;
    }
    if ((firmware_version < ADAPTER_MIN_FW_VERSION) ||
        ((firmware_version >> 8U) != (ADAPTER_API_VERSION >> 8U))) {
        libusb_close(device);
        libusb_exit(context);
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_INCOMPATIBLE_DEVICE;
    }

    (void)libusb_set_auto_detach_kernel_driver(device, 1);
    result = libusb_claim_interface(device, ADAPTER_USB_INTERFACE);
    if (result != LIBUSB_SUCCESS) {
        libusb_close(device);
        libusb_exit(context);
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_UNABLE_TO_OPEN;
    }
    if (pthread_mutex_init(&s_handles[slot].io_mutex, NULL) != 0) {
        (void)libusb_release_interface(device, ADAPTER_USB_INTERFACE);
        libusb_close(device);
        libusb_exit(context);
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_UNABLE_TO_OPEN;
    }
    s_handles[slot].context = context;
    s_handles[slot].device = device;
    s_handles[slot].next_sequence = 1U;
    (void)pthread_mutex_unlock(&s_registry_mutex);
    return (Aardvark)(slot + 1U);
}

static void store_le16(u08 *bytes, u16 value)
{
    bytes[0] = (u08)value;
    bytes[1] = (u08)(value >> 8U);
}

static void store_le32(u08 *bytes, u32 value)
{
    bytes[0] = (u08)value;
    bytes[1] = (u08)(value >> 8U);
    bytes[2] = (u08)(value >> 16U);
    bytes[3] = (u08)(value >> 24U);
}

static u16 load_le16(const u08 *bytes)
{
    return (u16)bytes[0] | ((u16)bytes[1] << 8U);
}

static u32 load_le32(const u08 *bytes)
{
    return (u32)bytes[0] |
           ((u32)bytes[1] << 8U) |
           ((u32)bytes[2] << 16U) |
           ((u32)bytes[3] << 24U);
}

static int api_send_all(libusb_device_handle *device,
                        const u08 *message, u16 message_length)
{
    u16 offset = 0U;

    while (offset < message_length) {
        int transferred = 0;
        int result = libusb_bulk_transfer(
            device, USB_PROTOCOL_EP_OUT,
            (unsigned char *)&message[offset], message_length - offset,
            &transferred, ADAPTER_USB_TIMEOUT_MS);

        if ((result != LIBUSB_SUCCESS) || (transferred <= 0)) {
            return STATUS_COMMUNICATION_ERROR;
        }
        offset = (u16)(offset + transferred);
    }
    return 0;
}

static int api_receive_response(libusb_device_handle *device,
                                u16 expected_opcode, u32 sequence,
                                u16 *response_status, u08 *payload,
                                u16 payload_capacity, u16 *payload_length)
{
    for (unsigned int stale = 0U;
         stale <= ADAPTER_USB_STALE_RESPONSE_LIMIT; stale++) {
        u08 message[USB_PROTOCOL_MAX_MESSAGE];
        u16 length = 0U;
        u16 expected_length = USB_PROTOCOL_HEADER_SIZE;

        while (length < expected_length) {
            u08 packet[ADAPTER_USB_PACKET_SIZE];
            int transferred = 0;
            int result = libusb_bulk_transfer(
                device, USB_PROTOCOL_EP_IN, packet, sizeof(packet),
                &transferred, ADAPTER_USB_TIMEOUT_MS);

            if (result != LIBUSB_SUCCESS) {
                return STATUS_COMMUNICATION_ERROR;
            }
            if (transferred == 0) {
                continue;
            }
            if ((u32)length + (u32)transferred > sizeof(message)) {
                return STATUS_COMMUNICATION_ERROR;
            }
            memcpy(&message[length], packet, (size_t)transferred);
            length = (u16)(length + transferred);
            if (length >= USB_PROTOCOL_HEADER_SIZE) {
                u16 received_payload_length;

                if (load_le32(&message[USB_HEADER_MAGIC_OFFSET]) !=
                    USB_PROTOCOL_MAGIC) {
                    return STATUS_COMMUNICATION_ERROR;
                }
                received_payload_length = load_le16(
                    &message[USB_HEADER_PAYLOAD_LENGTH_OFFSET]);
                if (received_payload_length > USB_PROTOCOL_MAX_PAYLOAD) {
                    return STATUS_COMMUNICATION_ERROR;
                }
                expected_length = (u16)(USB_PROTOCOL_HEADER_SIZE +
                                        received_payload_length);
            }
        }
        if ((length != expected_length) ||
            (message[USB_HEADER_VERSION_OFFSET] != USB_PROTOCOL_VERSION) ||
            (message[USB_HEADER_TYPE_OFFSET] != USB_MESSAGE_RESPONSE)) {
            return STATUS_COMMUNICATION_ERROR;
        }
        if ((load_le16(&message[USB_HEADER_OPCODE_OFFSET]) !=
             expected_opcode) ||
            (load_le32(&message[USB_HEADER_SEQUENCE_OFFSET]) != sequence)) {
            continue;
        }

        *response_status = load_le16(&message[USB_HEADER_STATUS_OFFSET]);
        *payload_length =
            load_le16(&message[USB_HEADER_PAYLOAD_LENGTH_OFFSET]);
        if (*payload_length > payload_capacity) {
            return STATUS_COMMUNICATION_ERROR;
        }
        if (*payload_length > 0U) {
            memcpy(payload, &message[USB_PROTOCOL_HEADER_SIZE],
                   *payload_length);
        }
        return 0;
    }
    return STATUS_COMMUNICATION_ERROR;
}

static int i2c_write_locked(AdapterHandle *handle, u16 slave_address,
                            AardvarkI2cFlags flags, u16 num_bytes,
                            const u08 *data)
{
    u08 message[USB_PROTOCOL_MAX_MESSAGE];
    u32 sequence;
    u16 response_status;
    u08 response[USB_I2C_WRITE_RESPONSE_SIZE];
    u16 response_length;
    u16 num_written;
    int result;

    sequence = handle->next_sequence++;
    if (handle->next_sequence == 0U) {
        handle->next_sequence = 1U;
    }
    store_le32(&message[USB_HEADER_MAGIC_OFFSET], USB_PROTOCOL_MAGIC);
    message[USB_HEADER_VERSION_OFFSET] = USB_PROTOCOL_VERSION;
    message[USB_HEADER_TYPE_OFFSET] = USB_MESSAGE_REQUEST;
    store_le16(&message[USB_HEADER_OPCODE_OFFSET], USB_OPCODE_I2C_WRITE);
    store_le32(&message[USB_HEADER_SEQUENCE_OFFSET], sequence);
    store_le16(&message[USB_HEADER_PAYLOAD_LENGTH_OFFSET],
               (u16)(USB_I2C_WRITE_PREFIX_SIZE + num_bytes));
    store_le16(&message[USB_HEADER_STATUS_OFFSET], 0U);
    message[USB_PROTOCOL_HEADER_SIZE] = (u08)slave_address;
    store_le16(&message[USB_PROTOCOL_HEADER_SIZE + 1U], flags);
    store_le16(&message[USB_PROTOCOL_HEADER_SIZE + 3U], num_bytes);
    if (num_bytes > 0U) {
        memcpy(&message[USB_PROTOCOL_HEADER_SIZE + USB_I2C_WRITE_PREFIX_SIZE],
               data, num_bytes);
    }

    result = api_send_all(
        handle->device, message,
        (u16)(USB_PROTOCOL_HEADER_SIZE + USB_I2C_WRITE_PREFIX_SIZE +
              num_bytes));
    if (result < 0) {
        return result;
    }
    result = api_receive_response(
        handle->device, USB_OPCODE_I2C_WRITE, sequence, &response_status,
        response, sizeof(response), &response_length);
    if (result < 0) {
        return result;
    }
    if ((response_status != USB_STATUS_OK) ||
        (response_length != USB_I2C_WRITE_RESPONSE_SIZE)) {
        return I2C_WRITE_ERROR;
    }
    num_written = load_le16(response);
    if (num_written != num_bytes) {
        return I2C_WRITE_ERROR;
    }
    return (int)num_written;
}

int i2c_write(Aardvark aardvark, u16 slave_address,
              AardvarkI2cFlags flags, u16 num_bytes,
              const u08 *data)
{
    AdapterHandle *handle = lock_adapter_handle(aardvark);
    int result;

    if (handle == NULL) {
        return STATUS_INVALID_HANDLE;
    }
    if ((slave_address > 0x7FU) || (flags != I2C_NO_FLAGS) ||
        (num_bytes > USB_I2C_WRITE_MAX_DATA_LENGTH) ||
        ((num_bytes > 0U) && (data == NULL))) {
        unlock_adapter_handle(handle);
        return I2C_WRITE_ERROR;
    }
    result = i2c_write_locked(handle, slave_address, flags, num_bytes, data);
    unlock_adapter_handle(handle);
    return result;
}

static int i2c_read_locked(AdapterHandle *handle, u16 slave_address,
                           AardvarkI2cFlags flags, u16 num_bytes, u08 *data)
{
    u08 message[USB_PROTOCOL_HEADER_SIZE + USB_I2C_READ_EXT_REQUEST_SIZE];
    u08 response[USB_PROTOCOL_MAX_PAYLOAD];
    u32 sequence;
    u16 response_status;
    u16 response_length;
    u16 num_read;
    int result;

    sequence = handle->next_sequence++;
    if (handle->next_sequence == 0U) {
        handle->next_sequence = 1U;
    }
    store_le32(&message[USB_HEADER_MAGIC_OFFSET], USB_PROTOCOL_MAGIC);
    message[USB_HEADER_VERSION_OFFSET] = USB_PROTOCOL_VERSION;
    message[USB_HEADER_TYPE_OFFSET] = USB_MESSAGE_REQUEST;
    store_le16(&message[USB_HEADER_OPCODE_OFFSET], USB_OPCODE_I2C_READ_EXT);
    store_le32(&message[USB_HEADER_SEQUENCE_OFFSET], sequence);
    store_le16(&message[USB_HEADER_PAYLOAD_LENGTH_OFFSET],
               USB_I2C_READ_EXT_REQUEST_SIZE);
    store_le16(&message[USB_HEADER_STATUS_OFFSET], 0U);
    message[USB_PROTOCOL_HEADER_SIZE] = (u08)slave_address;
    store_le16(&message[USB_PROTOCOL_HEADER_SIZE + 1U], flags);
    store_le16(&message[USB_PROTOCOL_HEADER_SIZE + 3U], num_bytes);

    result = api_send_all(handle->device, message, sizeof(message));
    if (result < 0) {
        return result;
    }
    result = api_receive_response(
        handle->device, USB_OPCODE_I2C_READ_EXT, sequence, &response_status,
        response, sizeof(response), &response_length);
    if (result < 0) {
        return result;
    }
    if ((response_status != USB_STATUS_OK) ||
        (response_length < USB_I2C_READ_EXT_PREFIX_SIZE)) {
        return I2C_READ_ERROR;
    }
    num_read = load_le16(&response[1]);
    if ((response[0] > USB_I2C_STATUS_LAST_DATA_ACK) ||
        (num_read > num_bytes) ||
        (response_length !=
         (u16)(USB_I2C_READ_EXT_PREFIX_SIZE + num_read))) {
        return I2C_READ_ERROR;
    }
    if (num_read > 0U) {
        memcpy(data, &response[USB_I2C_READ_EXT_PREFIX_SIZE], num_read);
    }
    return (int)num_read;
}

int i2c_read(Aardvark aardvark, u16 slave_address,
             AardvarkI2cFlags flags, u16 num_bytes, u08 *data)
{
    AdapterHandle *handle = lock_adapter_handle(aardvark);
    int result;

    if (handle == NULL) {
        return STATUS_INVALID_HANDLE;
    }
    if ((slave_address > 0x7FU) || (flags != I2C_NO_FLAGS) ||
        (num_bytes == 0U) ||
        (num_bytes > USB_I2C_READ_MAX_DATA_LENGTH) || (data == NULL)) {
        unlock_adapter_handle(handle);
        return I2C_READ_ERROR;
    }
    result = i2c_read_locked(handle, slave_address, flags, num_bytes, data);
    unlock_adapter_handle(handle);
    return result;
}

static int i2c_slave_write_locked(AdapterHandle *handle, u16 num_bytes,
                                  const u08 *data)
{
    u08 message[USB_PROTOCOL_HEADER_SIZE +
                USB_I2C_SLAVE_WRITE_PREFIX_SIZE +
                USB_I2C_SLAVE_WRITE_MAX_DATA_LENGTH];
    u08 response[USB_I2C_SLAVE_WRITE_RESPONSE_SIZE];
    u32 sequence;
    u16 response_status;
    u16 response_length;
    u16 accepted_length;
    int result;

    sequence = handle->next_sequence++;
    if (handle->next_sequence == 0U) {
        handle->next_sequence = 1U;
    }
    store_le32(&message[USB_HEADER_MAGIC_OFFSET], USB_PROTOCOL_MAGIC);
    message[USB_HEADER_VERSION_OFFSET] = USB_PROTOCOL_VERSION;
    message[USB_HEADER_TYPE_OFFSET] = USB_MESSAGE_REQUEST;
    store_le16(&message[USB_HEADER_OPCODE_OFFSET],
               USB_OPCODE_I2C_SLAVE_WRITE);
    store_le32(&message[USB_HEADER_SEQUENCE_OFFSET], sequence);
    store_le16(&message[USB_HEADER_PAYLOAD_LENGTH_OFFSET],
               (u16)(USB_I2C_SLAVE_WRITE_PREFIX_SIZE + num_bytes));
    store_le16(&message[USB_HEADER_STATUS_OFFSET], 0U);
    store_le16(&message[USB_PROTOCOL_HEADER_SIZE], num_bytes);
    memcpy(&message[USB_PROTOCOL_HEADER_SIZE +
                    USB_I2C_SLAVE_WRITE_PREFIX_SIZE], data, num_bytes);

    result = api_send_all(
        handle->device, message,
        (u16)(USB_PROTOCOL_HEADER_SIZE +
              USB_I2C_SLAVE_WRITE_PREFIX_SIZE + num_bytes));
    if (result < 0) {
        return result;
    }
    result = api_receive_response(
        handle->device, USB_OPCODE_I2C_SLAVE_WRITE, sequence,
        &response_status, response, sizeof(response), &response_length);
    if (result < 0) {
        return result;
    }
    if ((response_status != USB_STATUS_OK) ||
        (response_length != USB_I2C_SLAVE_WRITE_RESPONSE_SIZE)) {
        return I2C_SLAVE_WRITE_ERROR;
    }
    accepted_length = load_le16(response);
    if ((accepted_length == 0U) || (accepted_length > num_bytes)) {
        return I2C_SLAVE_WRITE_ERROR;
    }
    return (int)accepted_length;
}

int i2c_slave_write(Aardvark aardvark, u16 num_bytes, const u08 *data)
{
    AdapterHandle *handle = lock_adapter_handle(aardvark);
    int result;

    if (handle == NULL) {
        return STATUS_INVALID_HANDLE;
    }
    if ((num_bytes == 0U) ||
        (num_bytes > USB_I2C_SLAVE_WRITE_MAX_DATA_LENGTH) ||
        (data == NULL)) {
        unlock_adapter_handle(handle);
        return I2C_SLAVE_WRITE_ERROR;
    }
    result = i2c_slave_write_locked(handle, num_bytes, data);
    unlock_adapter_handle(handle);
    return result;
}

I2cQueue queue_create(Aardvark aardvark)
{
    size_t slot;

    (void)pthread_mutex_lock(&s_registry_mutex);
    if ((aardvark < 1) || ((unsigned int)aardvark > ADAPTER_MAX_HANDLES) ||
        (s_handles[(size_t)aardvark - 1U].device == NULL)) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_INVALID_HANDLE;
    }
    for (slot = 0U; slot < ADAPTER_MAX_QUEUES; slot++) {
        if (!atomic_load(&s_queues[slot].in_use)) {
            QueueHandle *queue = &s_queues[slot];

            queue->aardvark = aardvark;
            queue->command_count = 0U;
            atomic_store(&queue->state, QUEUE_STATE_IDLE);
            atomic_store(&queue->in_use, true);
            (void)pthread_mutex_unlock(&s_registry_mutex);
            return (I2cQueue)(slot + 1U);
        }
    }
    (void)pthread_mutex_unlock(&s_registry_mutex);
    return STATUS_QUEUE_FULL;
}

int queue_i2c_write(I2cQueue queue_handle, u16 slave_address,
                    AardvarkI2cFlags flags, u16 num_bytes,
                    const u08 *data)
{
    QueueHandle *queue;
    QueueCommand *command;
    u08 *data_copy = NULL;

    (void)pthread_mutex_lock(&s_registry_mutex);
    queue = queue_from_handle_locked(queue_handle);
    if (queue == NULL) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_INVALID_QUEUE;
    }
    if (atomic_load(&queue->state) != QUEUE_STATE_IDLE) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_QUEUE_BUSY;
    }
    if ((slave_address > 0x7FU) || (flags != I2C_NO_FLAGS) ||
        (num_bytes > USB_I2C_WRITE_MAX_DATA_LENGTH) ||
        ((num_bytes > 0U) && (data == NULL))) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return I2C_WRITE_ERROR;
    }
    if (queue->command_count == ADAPTER_QUEUE_MAX_COMMANDS) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_QUEUE_FULL;
    }
    if (num_bytes > 0U) {
        data_copy = malloc(num_bytes);
        if (data_copy == NULL) {
            (void)pthread_mutex_unlock(&s_registry_mutex);
            return STATUS_NO_MEMORY;
        }
        memcpy(data_copy, data, num_bytes);
    }

    command = &queue->commands[queue->command_count++];
    command->type = QUEUE_COMMAND_I2C_WRITE;
    command->slave_address = slave_address;
    command->flags = flags;
    command->num_bytes = num_bytes;
    command->data = data_copy;
    (void)pthread_mutex_unlock(&s_registry_mutex);
    return 0;
}

int queue_i2c_read(I2cQueue queue_handle, u16 slave_address,
                   AardvarkI2cFlags flags, u16 num_bytes, u08 *data)
{
    QueueHandle *queue;
    QueueCommand *command;

    (void)pthread_mutex_lock(&s_registry_mutex);
    queue = queue_from_handle_locked(queue_handle);
    if (queue == NULL) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_INVALID_QUEUE;
    }
    if (atomic_load(&queue->state) != QUEUE_STATE_IDLE) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_QUEUE_BUSY;
    }
    if ((slave_address > 0x7FU) || (flags != I2C_NO_FLAGS) ||
        (num_bytes == 0U) ||
        (num_bytes > USB_I2C_READ_MAX_DATA_LENGTH) || (data == NULL)) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return I2C_READ_ERROR;
    }
    if (queue->command_count == ADAPTER_QUEUE_MAX_COMMANDS) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_QUEUE_FULL;
    }

    command = &queue->commands[queue->command_count++];
    command->type = QUEUE_COMMAND_I2C_READ;
    command->slave_address = slave_address;
    command->flags = flags;
    command->num_bytes = num_bytes;
    command->data = data;
    (void)pthread_mutex_unlock(&s_registry_mutex);
    return 0;
}

int queue_submit(I2cQueue queue_handle)
{
    QueueHandle *queue;
    AdapterHandle *handle;
    int expected = QUEUE_STATE_IDLE;
    int completed = 0;
    int final_result;

    (void)pthread_mutex_lock(&s_registry_mutex);
    queue = queue_from_handle_locked(queue_handle);
    if (queue == NULL) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_INVALID_QUEUE;
    }
    if (!atomic_compare_exchange_strong(&queue->state, &expected,
                                        QUEUE_STATE_SUBMITTING)) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_QUEUE_BUSY;
    }
    if ((queue->aardvark < 1) ||
        ((unsigned int)queue->aardvark > ADAPTER_MAX_HANDLES) ||
        (s_handles[(size_t)queue->aardvark - 1U].device == NULL)) {
        atomic_store(&queue->state, QUEUE_STATE_IDLE);
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_INVALID_HANDLE;
    }
    handle = &s_handles[(size_t)queue->aardvark - 1U];
    (void)pthread_mutex_lock(&handle->io_mutex);
    (void)pthread_mutex_unlock(&s_registry_mutex);

    final_result = completed;

    for (size_t index = 0U; index < queue->command_count; index++) {
        QueueCommand *command = &queue->commands[index];
        int result;

        if (atomic_load(&queue->state) == QUEUE_STATE_ABORT_REQUESTED) {
            final_result = STATUS_ABORTED;
            break;
        }
        if (command->type == QUEUE_COMMAND_I2C_WRITE) {
            result = i2c_write_locked(handle, command->slave_address,
                                      command->flags, command->num_bytes,
                                      command->data);
        } else {
            result = i2c_read_locked(handle, command->slave_address,
                                     command->flags, command->num_bytes,
                                     command->data);
        }
        if (result < 0) {
            final_result = result;
            break;
        }
        completed++;
        final_result = completed;
    }

    atomic_store(&queue->state, QUEUE_STATE_IDLE);
    (void)pthread_mutex_unlock(&handle->io_mutex);
    return final_result;
}

int async_abort(I2cQueue queue_handle)
{
    QueueHandle *queue;

    if ((queue_handle < 1) ||
        ((unsigned int)queue_handle > ADAPTER_MAX_QUEUES)) {
        return STATUS_INVALID_QUEUE;
    }
    queue = &s_queues[(size_t)queue_handle - 1U];
    if (!atomic_load(&queue->in_use)) {
        return STATUS_INVALID_QUEUE;
    }
    {
        int expected = QUEUE_STATE_SUBMITTING;

        (void)atomic_compare_exchange_strong(
            &queue->state, &expected, QUEUE_STATE_ABORT_REQUESTED);
    }
    return 0;
}

int queue_clear(I2cQueue queue_handle)
{
    QueueHandle *queue;

    (void)pthread_mutex_lock(&s_registry_mutex);
    queue = queue_from_handle_locked(queue_handle);
    if (queue == NULL) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_INVALID_QUEUE;
    }
    if (atomic_load(&queue->state) != QUEUE_STATE_IDLE) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_QUEUE_BUSY;
    }
    clear_queue_commands(queue);
    (void)pthread_mutex_unlock(&s_registry_mutex);
    return 0;
}

int queue_destroy(I2cQueue queue_handle)
{
    QueueHandle *queue;

    (void)pthread_mutex_lock(&s_registry_mutex);
    queue = queue_from_handle_locked(queue_handle);
    if (queue == NULL) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_INVALID_QUEUE;
    }
    if (atomic_load(&queue->state) != QUEUE_STATE_IDLE) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_QUEUE_BUSY;
    }
    clear_queue_commands(queue);
    queue->aardvark = 0;
    atomic_store(&queue->in_use, false);
    atomic_store(&queue->state, QUEUE_STATE_IDLE);
    (void)pthread_mutex_unlock(&s_registry_mutex);
    return 0;
}

int adapter_close(Aardvark aardvark)
{
    (void)pthread_mutex_lock(&s_registry_mutex);
    if (aardvark == 0) {
        int closed = 0;

        for (size_t slot = 0U; slot < ADAPTER_MAX_HANDLES; slot++) {
            if ((s_handles[slot].device != NULL) &&
                adapter_queue_is_busy((Aardvark)(slot + 1U))) {
                (void)pthread_mutex_unlock(&s_registry_mutex);
                return STATUS_QUEUE_BUSY;
            }
        }

        for (size_t slot = 0U; slot < ADAPTER_MAX_HANDLES; slot++) {
            if (s_handles[slot].device != NULL) {
                (void)pthread_mutex_lock(&s_handles[slot].io_mutex);
                release_queues_for_adapter((Aardvark)(slot + 1U));
                close_handle_slot(slot);
                (void)pthread_mutex_unlock(&s_handles[slot].io_mutex);
                (void)pthread_mutex_destroy(&s_handles[slot].io_mutex);
                closed++;
            }
        }
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return closed;
    }
    if ((aardvark < 1) || ((unsigned int)aardvark > ADAPTER_MAX_HANDLES) ||
        (s_handles[(size_t)aardvark - 1U].device == NULL)) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_INVALID_HANDLE;
    }
    if (adapter_queue_is_busy(aardvark)) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return STATUS_QUEUE_BUSY;
    }
    (void)pthread_mutex_lock(
        &s_handles[(size_t)aardvark - 1U].io_mutex);
    release_queues_for_adapter(aardvark);
    close_handle_slot((size_t)aardvark - 1U);
    (void)pthread_mutex_unlock(
        &s_handles[(size_t)aardvark - 1U].io_mutex);
    (void)pthread_mutex_destroy(
        &s_handles[(size_t)aardvark - 1U].io_mutex);
    (void)pthread_mutex_unlock(&s_registry_mutex);
    return 1;
}

libusb_device_handle *adapter_libusb_handle(Aardvark aardvark)
{
    libusb_device_handle *device;

    (void)pthread_mutex_lock(&s_registry_mutex);
    if ((aardvark < 1) || ((unsigned int)aardvark > ADAPTER_MAX_HANDLES)) {
        (void)pthread_mutex_unlock(&s_registry_mutex);
        return NULL;
    }
    device = s_handles[(size_t)aardvark - 1U].device;
    (void)pthread_mutex_unlock(&s_registry_mutex);
    return device;
}

libusb_device_handle *adapter_open_port(libusb_context *context, u16 port)
{
    libusb_device **list = NULL;
    libusb_device **matches = NULL;
    libusb_device_handle *handle = NULL;
    ssize_t list_length;
    u16 wanted = (u16)(port & ~ADAPTER_PORT_NOT_FREE);
    int found = 0;

    if (context == NULL) {
        return NULL;
    }
    list_length = libusb_get_device_list(context, &list);
    if (list_length < 0) {
        return NULL;
    }
    matches = calloc((size_t)list_length, sizeof(*matches));
    if (matches == NULL) {
        libusb_free_device_list(list, 1);
        return NULL;
    }
    for (ssize_t index = 0; index < list_length; index++) {
        if (is_adapter(list[index])) {
            matches[found++] = list[index];
        }
    }
    qsort(matches, (size_t)found, sizeof(*matches), compare_devices);
    if ((int)wanted < found) {
        if (libusb_open(matches[wanted], &handle) != LIBUSB_SUCCESS) {
            handle = NULL;
        }
    }

    free(matches);
    libusb_free_device_list(list, 1);
    return handle;
}
