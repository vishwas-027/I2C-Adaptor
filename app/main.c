#include "../board/board.h"
#include "../drivers/led.h"
#include "../drivers/uart.h"
#include "../drivers/i2c.h"
#include "../middleware/logging/logger.h"

#include "app_tasks.h"

#include "../middleware/RTOS/include/FreeRTOS.h"
#include "../middleware/RTOS/include/task.h"

int main(void)
{
    /* Hardware must be ready before middleware or FreeRTOS tasks use it. */
    boardInit();
    led_init();
    uart_init(115200);
    logger_init();

    /* Creates the LED status task and the interactive SMBus CLI task. */
    app_create_tasks();

    /* Control does not return unless the FreeRTOS heap is misconfigured. */
    vTaskStartScheduler();

    while (1)
    {
    }
}
