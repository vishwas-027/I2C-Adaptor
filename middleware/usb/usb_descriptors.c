#include "tusb.h"
#include "stm32f411xe.h"

#include <stddef.h>
#include <string.h>

/* Development-only VID/PID. These must be replaced by legitimately assigned
 * identifiers before the product is distributed. */
#define USB_VID                 0xCAFEU
#define USB_PID                 0x4110U
#define USB_BCD_DEVICE          0x0100U
#define USB_EP_SIZE             64U
#define USB_EP_VENDOR_OUT       0x01U
#define USB_EP_VENDOR_IN        0x81U

enum {
    USB_ITF_VENDOR = 0,
    USB_ITF_COUNT
};

enum {
    USB_STR_LANGUAGE = 0,
    USB_STR_MANUFACTURER,
    USB_STR_PRODUCT,
    USB_STR_SERIAL,
    USB_STR_VENDOR_INTERFACE
};

static const tusb_desc_device_t s_device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = USB_BCD_DEVICE,
    .iManufacturer = USB_STR_MANUFACTURER,
    .iProduct = USB_STR_PRODUCT,
    .iSerialNumber = USB_STR_SERIAL,
    .bNumConfigurations = 1
};

#define USB_CONFIG_TOTAL_LENGTH (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN)

static const uint8_t s_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, USB_ITF_COUNT, 0, USB_CONFIG_TOTAL_LENGTH,
                          0x00, 100),
    TUD_VENDOR_DESCRIPTOR(USB_ITF_VENDOR, USB_STR_VENDOR_INTERFACE,
                          USB_EP_VENDOR_OUT, USB_EP_VENDOR_IN, USB_EP_SIZE)
};

static const char *const s_string_descriptors[] = {
    (const char[]){ 0x09, 0x04 },
    "SMBus Adapter Project",
    "STM32F411 SMBus Adapter",
    NULL,
    "Vendor Bulk Interface"
};

static uint16_t s_string_descriptor[32];

static void usb_serial_number(char serial[11])
{
    const volatile uint32_t *uid = (const volatile uint32_t *)UID_BASE;
    uint32_t hash = 2166136261U;

    for (size_t word = 0U; word < 3U; word++) {
        uint32_t value = uid[word];

        for (size_t byte = 0U; byte < sizeof(value); byte++) {
            hash ^= value & 0xFFU;
            hash *= 16777619U;
            value >>= 8U;
        }
    }
    if (hash == 0U) {
        hash = 1U;
    }
    for (size_t digit = 10U; digit > 0U; digit--) {
        serial[digit - 1U] = (char)('0' + (hash % 10U));
        hash /= 10U;
    }
    serial[10] = '\0';
}

const uint8_t *tud_descriptor_device_cb(void)
{
    return (const uint8_t *)&s_device_descriptor;
}

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return s_configuration_descriptor;
}

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t language_id)
{
    size_t character_count;

    (void)language_id;
    if (index >= (sizeof(s_string_descriptors) /
                  sizeof(s_string_descriptors[0]))) {
        return NULL;
    }

    if (index == USB_STR_LANGUAGE) {
        memcpy(&s_string_descriptor[1], s_string_descriptors[0], 2U);
        character_count = 1U;
    } else {
        char serial[11];
        const char *text = s_string_descriptors[index];

        if (index == USB_STR_SERIAL) {
            usb_serial_number(serial);
            text = serial;
        }
        character_count = strlen(text);
        if (character_count > 31U) {
            character_count = 31U;
        }
        for (size_t i = 0U; i < character_count; i++) {
            s_string_descriptor[1U + i] = (uint8_t)text[i];
        }
    }

    s_string_descriptor[0] =
        (uint16_t)((TUSB_DESC_STRING << 8U) | (2U * character_count + 2U));
    return s_string_descriptor;
}
