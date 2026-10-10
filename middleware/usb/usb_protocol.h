#ifndef USB_PROTOCOL_DEVICE_H
#define USB_PROTOCOL_DEVICE_H

/* Initialize/reset the firmware-side stream parser and response transmitter. */
void usb_protocol_init(void);
void usb_protocol_reset(void);

/* Drain received vendor bytes and advance pending response transmission.
 * Called only from the TinyUSB task, never from interrupt context. */
void usb_protocol_task(void);

#endif /* USB_PROTOCOL_DEVICE_H */
