/* Public interface for the UART command-line interface. The CLI owns SMBus
 * command handling and exposes only its lifecycle and connection state. */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdbool.h>

/* Initialise the console-owned SMBus resources before the scheduler starts. */
void console_init(void);

/* FreeRTOS task entry point for the UART command console. */
void console_task(void *argument);

/* True when the most recent SMBus transaction was acknowledged. */
bool console_smbus_is_connected(void);

#endif /* CONSOLE_H */
