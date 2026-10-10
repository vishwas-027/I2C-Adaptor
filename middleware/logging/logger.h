/******************************************************************************
 * @file    logger.h
 * @brief   Logging Middleware
 *
 * Public interface for the logging subsystem.
 *
 * The application should use the LOG_xxx() macros instead of calling
 * logger_printf() directly.
 ******************************************************************************/

#ifndef LOGGER_H
#define LOGGER_H

/******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdarg.h>
#include <stdint.h>

#include "logger_cfg.h"

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/*
 * Initialize the logger.
 */
void logger_init(void);

/*
 * Internal logging function.
 *
 * Applications should use LOG_INFO(), LOG_ERROR(), etc.
 */
void logger_log(logger_level_t level,
                const char *file,
                uint32_t line,
                const char *function,
                const char *fmt,
                ...);

/******************************************************************************
 * Logging Macros
 ******************************************************************************/

#define LOG_DEBUG(fmt, ...)                                     \
    logger_log(LOGGER_LEVEL_DEBUG,                              \
               __FILE__,                                        \
               __LINE__,                                        \
               __func__,                                        \
               fmt,                                             \
               ##__VA_ARGS__)

#define LOG_INFO(fmt, ...)                                      \
    logger_log(LOGGER_LEVEL_INFO,                               \
               __FILE__,                                        \
               __LINE__,                                        \
               __func__,                                        \
               fmt,                                             \
               ##__VA_ARGS__)

#define LOG_WARN(fmt, ...)                                      \
    logger_log(LOGGER_LEVEL_WARN,                               \
               __FILE__,                                        \
               __LINE__,                                        \
               __func__,                                        \
               fmt,                                             \
               ##__VA_ARGS__)

#define LOG_ERROR(fmt, ...)                                     \
    logger_log(LOGGER_LEVEL_ERROR,                              \
               __FILE__,                                        \
               __LINE__,                                        \
               __func__,                                        \
               fmt,                                             \
               ##__VA_ARGS__)

#define LOG_FATAL(fmt, ...)                                     \
    logger_log(LOGGER_LEVEL_FATAL,                              \
               __FILE__,                                        \
               __LINE__,                                        \
               __func__,                                        \
               fmt,                                             \
               ##__VA_ARGS__)

#endif /* LOGGER_H */