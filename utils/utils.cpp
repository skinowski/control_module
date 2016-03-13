/*
 * Copyright (C) 2016 Tolga Ceylan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 */
#include "utils.h"
#include <errno.h>
#include <dirent.h>
#include <string.h>
#include <stdio.h>
#include "logger.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


namespace robo {

uint64_t soundspeed_usec2cm(uint64_t usec, float temp)
{
    // TODO: adjust for temperature!
    // The speed of sound is 340 m/s or 29 microseconds per centimeter.
    // The ping travels out and back, so to find the distance of the
    // object we take half of the distance travelled.
    // http://www.parallax.com/sites/default/files/downloads/28015-PING-Documentation-v1.6.pdf
    const float speed = 331.5f + (0.6f * temp); // m/sec
    return usec / (10000.0f / speed);
}







} // namespace robo