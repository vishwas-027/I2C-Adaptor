/******************************************************************************
 * @file    logger.c
 * @brief   Logging Middleware
 ******************************************************************************/

/******************************************************************************
 * Includes
 ******************************************************************************/

#include "logger.h"
#include "logger_cfg.h"
#include "backend/logger_backend.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/******************************************************************************
 * Private Macros
 ******************************************************************************/

#define LOGGER_PREFIX_MAX_LENGTH      64U

/******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

static const char *logger_level_to_string(logger_level_t level);

static const char *logger_level_to_color(logger_level_t level);


/******************************************************************************
 * Public Functions
 ******************************************************************************/

void logger_init(void)
{
    logger_backend_init();
}

void logger_log(logger_level_t level,
                const char *file,
                uint32_t line,
                const char *function,
                const char *fmt,
                ...)
{
    (void)line;
    (void)function;
    (void)file;

    const int minimum_level = (int)LOGGER_LOG_LEVEL;

    if ((int)level < minimum_level)
    {
        return;
    }

    char message[LOGGER_MAX_MESSAGE_LENGTH];
    char prefix[LOGGER_PREFIX_MAX_LENGTH];

    memset(message, 0, sizeof(message));
    memset(prefix, 0, sizeof(prefix));

    /**********************************************************************
     * Prefix
     *********************************************************************/

#if LOGGER_ENABLE_COLOR

    snprintf(prefix,
             sizeof(prefix),
             "%s[%s]\033[0m ",
             logger_level_to_color(level),
             logger_level_to_string(level));

#else

    snprintf(prefix,
             sizeof(prefix),
             "[%s] ",
             logger_level_to_string(level));

#endif

    /**********************************************************************
     * Format user message
     *********************************************************************/

    va_list args;

    va_start(args, fmt);

    vsnprintf(message,
              sizeof(message),
              fmt,
              args);

    va_end(args);

    /**********************************************************************
     * Output
     *********************************************************************/

    logger_backend_write(
        (const uint8_t *)prefix,
        strlen(prefix));

    logger_backend_write(
        (const uint8_t *)message,
        strlen(message));

    logger_backend_write(
        (const uint8_t *)"\r\n",
        2U);
}

/******************************************************************************
 * Private Functions
 ******************************************************************************/

static const char *logger_level_to_string(logger_level_t level)
{
    switch (level)
    {
        case LOGGER_LEVEL_DEBUG:
            return "DEBUG";

        case LOGGER_LEVEL_INFO:
            return "INFO ";

        case LOGGER_LEVEL_WARN:
            return "WARN ";

        case LOGGER_LEVEL_ERROR:
            return "ERROR";

        case LOGGER_LEVEL_FATAL:
            return "FATAL";

        default:
            return "UNKWN";
    }
}

static const char *logger_level_to_color(logger_level_t level)
{
    switch (level)
    {
        case LOGGER_LEVEL_DEBUG:
            return "\033[36m";

        case LOGGER_LEVEL_INFO:
            return "\033[32m";

        case LOGGER_LEVEL_WARN:
            return "\033[33m";

        case LOGGER_LEVEL_ERROR:
            return "\033[31m";

        case LOGGER_LEVEL_FATAL:
            return "\033[35m";

        default:
            return "\033[0m";
    }
}