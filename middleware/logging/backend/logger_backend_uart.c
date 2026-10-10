/******************************************************************************
 * @file    logger_backend_uart.c
 * @brief   UART Backend for Logger
 *
 * This file adapts the generic logger backend interface to the UART driver.
 *
 * The logger knows nothing about UART registers or USART peripherals.
 * Likewise, the UART driver knows nothing about the logger.
 ******************************************************************************/

#include <stdint.h>
#include <stddef.h>

#include "../../drivers/uart.h"

#include "logger_backend.h"

void logger_backend_init(void)
{
    /* UART is already initialized by board/application.
     *
     * If desired, this function can call:
     *
     *      uart_init(115200);
     *
     * For now we keep initialization responsibility in boardInit()/main().
     */
}

void logger_backend_write(const uint8_t *buffer, uint32_t length)
{
    if ((buffer == NULL) || (length == 0U))
    {
        return;
    }

    uart_write(buffer, length);
}