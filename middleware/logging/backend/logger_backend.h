/******************************************************************************
 * @file    logger_backend.h
 * @brief   Logger Backend Interface
 ******************************************************************************/

#ifndef LOGGER_BACKEND_H
#define LOGGER_BACKEND_H

#include <stdint.h>

/******************************************************************************
 * Public Interface
 ******************************************************************************/

/*
 * Initialize backend.
 *
 * Examples:
 *      UART
 *      USB CDC
 *      SWO
 *      RTT
 */
void logger_backend_init(void);

/*
 * Write raw bytes.
 */
void logger_backend_write(
    const uint8_t *buffer,
    uint32_t length);

#endif /* LOGGER_BACKEND_H */