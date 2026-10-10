#ifndef USB_DEVICE_H
#define USB_DEVICE_H

#include <stdbool.h>

/* FreeRTOS task that initializes and services TinyUSB device events. */
void usb_device_task(void *argument);

/* True after the host has selected this device configuration. */
bool usb_device_is_mounted(void);

#endif /* USB_DEVICE_H */
