#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

/* TinyUSB runs in its own FreeRTOS task. The DWC2 interrupt only posts
 * events; descriptor handling and application callbacks run in task context. */
#define CFG_TUSB_OS                    OPT_OS_FREERTOS
#define CFG_TUSB_OS_INC_PATH
#define CFG_TUSB_DEBUG                 0

#define CFG_TUD_ENABLED                1
#define CFG_TUH_ENABLED                0
#define CFG_TUSB_RHPORT0_MODE          (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUD_MAX_SPEED              TUSB_SPEED_FULL

/* The Black Pill USB connector does not route VBUS to PA9. TinyUSB will
 * therefore use the STM32 DWC2 software B-session-valid override. */
#define CFG_TUD_VBUS_DETECT_HW         0

#define CFG_TUD_ENDPOINT0_SIZE         64

#define CFG_TUD_CDC                    0
#define CFG_TUD_MSC                    0
#define CFG_TUD_HID                    0
#define CFG_TUD_MIDI                   0
#define CFG_TUD_AUDIO                  0
#define CFG_TUD_VENDOR                 1

#define CFG_TUD_VENDOR_RX_BUFSIZE      64
#define CFG_TUD_VENDOR_TX_BUFSIZE      64

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN             __attribute__((aligned(4)))

#endif /* TUSB_CONFIG_H */
