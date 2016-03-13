/*
 * Copyright (C) 2016 Tolga Ceylan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 */

#ifndef LOGGER_H_
#define LOGGER_H_

namespace robo {

enum LogLevel
{
    LOG_TRACE,
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR,
};

void logger(LogLevel logLevel, const char *format, ...);
const char *get_log_level_str(LogLevel logLevel);

} // namespace robo

#endif /* LOGGER_H_ */
