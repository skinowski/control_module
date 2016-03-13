/*
 * Copyright (C) 2016 Tolga Ceylan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 * This file incorporates work covered by the following copyright and  
 * permission notice:
 */
    /*
    UtilTime.cpp provides time functions
    Copyright (C) 2014 Intel Corporation
    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.
    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
    */
#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>
#include <limits>
#include <unistd.h>
#include <time.h>

namespace robo {

class Timer
{
public:
    Timer();

    static int initialize();

    // From Arduino-IDE source in hardware/arduino/edison/cores/arduino/UtilTime.cpp
    static uint64_t rdtsc();

    // Ported from Arduino-IDE source in hardware/arduino/edison/cores/arduino/UtilTime.cpp
    static inline uint64_t get_elapsed_usec(uint64_t prev, uint64_t now)
    {
        /*
        This function returns a 64-bit value representing the microseconds
        since the last time update() was called.

        overflow should not be a problem, though for
        correctness it should be accounted for.
        A 64 bit counter at 400mhz will run for about
        1500 years before it overflows. the timestamp
        counter on IA32 is a 64-bit counter that ticks at
        the CPU clock rate, starting at 0.
        */
        if (now >= prev)
        {
            const uint64_t diff = now - prev;
            return diff / s_cpufreq;
        }
        return std::numeric_limits<uint64_t>::max() - (prev - now) / s_cpufreq;
    }

    uint64_t get_now() const
    {
        return Timer::rdtsc();
    }

    void set_start(uint64_t now)
    {
        m_start = now;
    }

    uint64_t get_start() const
    {
        return m_start;
    }

    static uint64_t get_cpu_freq()
    {
        return s_cpufreq;
    }

private:
    uint64_t m_start;
    static uint64_t s_cpufreq;
};


} // namespace robo

#endif /* TIMER_H_ */
