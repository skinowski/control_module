/*
 * timer.h
 *
 *  Created on: Dec 21, 2014
 *      Author: tceylan
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
