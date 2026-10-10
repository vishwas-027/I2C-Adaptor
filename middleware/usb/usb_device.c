#include "usb_device.h"
#include "usb_protocol.h"

#include "FreeRTOS.h"
#include "logger.h"
#include "tusb.h"

#include "../../include/stm32f411.h"

#include <stdint.h>

#define USB_DM_PIN              11U
#define USB_DP_PIN              12U
#define USB_GPIO_AF             10U
#define USB_OTG_FS_IRQ_NUMBER   67U
#define USB_IRQ_PRIORITY        6U
#define NVIC_IPR_BASE           0xE000E400UL
#define USB_OTG_FS_DCTL         (*(volatile uint32_t *)0x50000804UL)
#define USB_OTG_DCTL_SDIS       (1UL << 1)

static volatile bool s_usb_mounted;

static void usb_gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* PA11/PA12: OTG_FS_DM/DP, alternate function 10, high speed. */
    GPIOA->MODER &= ~((3UL << (USB_DM_PIN * 2U)) |
                      (3UL << (USB_DP_PIN * 2U)));
    GPIOA->MODER |=  ((2UL << (USB_DM_PIN * 2U)) |
                      (2UL << (USB_DP_PIN * 2U)));
    GPIOA->OTYPER &= ~((1UL << USB_DM_PIN) | (1UL << USB_DP_PIN));
    GPIOA->OSPEEDR |= ((3UL << (USB_DM_PIN * 2U)) |
                       (3UL << (USB_DP_PIN * 2U)));
    GPIOA->PUPDR &= ~((3UL << (USB_DM_PIN * 2U)) |
                      (3UL << (USB_DP_PIN * 2U)));

    GPIOA->AFR[1] &= ~((0xFUL << ((USB_DM_PIN - 8U) * 4U)) |
                       (0xFUL << ((USB_DP_PIN - 8U) * 4U)));
    GPIOA->AFR[1] |=  ((USB_GPIO_AF << ((USB_DM_PIN - 8U) * 4U)) |
                       (USB_GPIO_AF << ((USB_DP_PIN - 8U) * 4U)));

    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    (void)RCC->AHB2ENR;
}

static void usb_irq_priority_init(void)
{
    volatile uint8_t *priority = (volatile uint8_t *)NVIC_IPR_BASE;

    /* With four implemented priority bits and a FreeRTOS syscall ceiling of
     * five, numerical priority six may safely call the FromISR queue APIs. */
    priority[USB_OTG_FS_IRQ_NUMBER] =
        (uint8_t)(USB_IRQ_PRIORITY << (8U - __NVIC_PRIO_BITS));
}

void usb_device_task(void *argument)
{
    tusb_rhport_init_t device_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL
    };
    tud_configure_dwc2_t dwc2_config = CFG_TUD_CONFIGURE_DWC2_DEFAULT;

    (void)argument;
    s_usb_mounted = false;

    usb_gpio_init();
    usb_irq_priority_init();
    usb_protocol_init();

    /* SWD flashing can leave the old D+ pull-up visible throughout reset,
     * preventing the host from noticing that new firmware has started.
     * Hold a real soft-disconnect long enough for every host to observe it;
     * TinyUSB clears SDIS when dcd_init() connects below. */
    USB_OTG_FS_DCTL |= USB_OTG_DCTL_SDIS;
    vTaskDelay(pdMS_TO_TICKS(25U));

    /* Must be configured before tusb_init(); the Black Pill has no VBUS
     * sensing connection, so force device-session validity in the DWC2 core. */
    dwc2_config.vbus_sensing = false;
    configASSERT(tud_configure(0U, TUD_CFGID_DWC2, &dwc2_config));
    configASSERT(tusb_init(0U, &device_init));

    LOG_INFO("USB: TinyUSB vendor device initialized");

    for (;;) {
        /* In FreeRTOS mode this blocks on TinyUSB's ISR-to-task event queue. */
        tud_task();
        usb_protocol_task();
    }
}

bool usb_device_is_mounted(void)
{
    return s_usb_mounted;
}

void OTG_FS_IRQHandler(void)
{
    tusb_int_handler(0U, true);
}

void tud_mount_cb(void)
{
    s_usb_mounted = true;
    usb_protocol_reset();
    LOG_INFO("USB: mounted by host");
}

void tud_umount_cb(void)
{
    s_usb_mounted = false;
    usb_protocol_reset();
    LOG_INFO("USB: unmounted from host");
}

void tud_suspend_cb(bool remote_wakeup_enabled)
{
    (void)remote_wakeup_enabled;
    LOG_INFO("USB: suspended");
}

void tud_resume_cb(void)
{
    LOG_INFO("USB: resumed");
}
