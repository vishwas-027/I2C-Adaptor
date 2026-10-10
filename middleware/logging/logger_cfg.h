/******************************************************************************
 * @file    logger_cfg.h
 * @brief   Logger Configuration
 *
 * Modify only this file to configure the logging middleware.
 ******************************************************************************/

#ifndef LOGGER_CFG_H
#define LOGGER_CFG_H

#include <stdint.h>

/******************************************************************************
 * Log Levels
 ******************************************************************************/

typedef enum
{
    LOGGER_LEVEL_DEBUG = 0,
    LOGGER_LEVEL_INFO,
    LOGGER_LEVEL_WARN,
    LOGGER_LEVEL_ERROR,
    LOGGER_LEVEL_FATAL

} logger_level_t;

/******************************************************************************
 * Configuration
 ******************************************************************************/

/* Maximum formatted message length */
#define LOGGER_MAX_MESSAGE_LENGTH      256U

/* Compile-time log filtering */
#define LOGGER_LOG_LEVEL               LOGGER_LEVEL_DEBUG

/* Output formatting */
#define LOGGER_ENABLE_COLOR            1U
#define LOGGER_ENABLE_TIMESTAMP        1U
#define LOGGER_ENABLE_MODULE_NAME      0U
#define LOGGER_ENABLE_FILE_NAME        0U
#define LOGGER_ENABLE_LINE_NUMBER      0U

#endif /* LOGGER_CFG_H */