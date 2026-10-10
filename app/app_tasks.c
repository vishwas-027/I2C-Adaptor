#include "FreeRTOS.h"
#include "task.h"

#include "../drivers/led.h"
#include "../middleware/cli/console.h"
#include "../middleware/usb/usb_device.h"
#include "app_tasks.h"

static void LedTask(void *argument)
{
    (void)argument;

    /* PC13 is a steady status indicator, not a heartbeat: the console
     * records whether its latest SMBus transaction was acknowledged. */
    while (1) {
        if (console_smbus_is_connected()) {
            led_on();
        } else {
            led_off();
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_create_tasks(void)
{
    BaseType_t status;

    /* The CLI owns I2C1 and creates the mutex used around every transfer. */
    console_init();
    led_off();

    /* Keep both tasks at the same priority; the console blocks while waiting
     * for UART input, allowing the status LED task to run regularly. */
    status = xTaskCreate(LedTask, "LED", 256, NULL, 1, NULL);
    configASSERT(status == pdPASS);

    status = xTaskCreate(console_task, "CONSOLE", 512, NULL, 1, NULL);
    configASSERT(status == pdPASS);

    /* USB runs above the application tasks so enumeration and endpoint
     * events are drained promptly even while a CLI command is executing. */
    status = xTaskCreate(usb_device_task, "USB", 1024, NULL, 3, NULL);
    configASSERT(status == pdPASS);
}
