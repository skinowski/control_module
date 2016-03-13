/*
 * timer.cpp
 *
 *  Created on: Dec 21, 2014
 *      Author: tceylan
 */

#include "timer.h"
#include "logger.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> // O_RDONLY
#include <math.h> // HUGE_VAL
#include <errno.h>
#include <string.h> // memset
#include <unistd.h> // read
#include <stdlib.h> // strtod
#include <assert.h>

namespace robo {

uint64_t Timer::s_cpufreq = 0;

// From Arduino-IDE source in hardware/arduino/edison/cores/arduino/UtilTime.cpp
uint64_t Timer::rdtsc()
{
    uint64_t returnVal = 0;

#if defined(__x86_64__) || defined(__amd64__)
    uint32_t lo = 0;
    uint32_t hi = 0;
    /* We cannot use "=A", since this would use %rax on x86_64 */
    __asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
    returnVal = hi;
    returnVal <<= 32;
    returnVal |= lo;

#else
    struct timespec tp;
    tp.tv_sec = 0;
    tp.tv_nsec = 0;
    const int res = ::clock_gettime(CLOCK_MONOTONIC, &tp);
    if (res)
    {
        logger(LOG_ERROR, "::clock_gettime error err=%d", errno);
        assert(false);
        return 0;
    }
    
    returnVal = tp.tv_sec * 1000000;
    returnVal += tp.tv_nsec / 1000;
#endif
    return returnVal;
}


Timer::Timer()
	:
	m_start(0)
{
}

// Ported from Arduino-IDE source in hardware/arduino/edison/cores/arduino/UtilTime.cpp
int Timer::initialize()
{
	if (s_cpufreq != 0)
		return EINVAL;

#if defined(__arm__)
        s_cpufreq = 1;
        return 0;
#else

    char buf[0x400];
    const char * const mhz_str = "cpu MHz\t\t: ";

    const int cpufreq_fd = open("/proc/cpuinfo", O_RDONLY);
    if( cpufreq_fd < 0)
    {
    	logger(LOG_ERROR, "unable to open /proc/cpuinfo err=%d", errno);
        return errno;
    }
    memset(buf, 0x00, sizeof(buf));
    const int ret = read(cpufreq_fd, buf, sizeof(buf));
    if ( ret < 0 )
    {
    	logger(LOG_ERROR, "unable to read cpuinfo err=%d!", errno);
        close(cpufreq_fd);
        return errno;
    }
    close(cpufreq_fd);
    char *str = strstr(buf, mhz_str);
    if (!str)
    {
    	logger(LOG_ERROR, "Buffer %s does not contain CPU frequency info !", buf);
        return EFAULT;
    }

    str += strlen(mhz_str);
    char *str2 = str;

    while(str2 < buf  + sizeof(buf)-1 && *str2 != '\n')
        str2++;

    if(str2 == buf + sizeof(buf-1) && *str2 !='\n')
    {
        logger(LOG_ERROR, "malformed cpufreq string %s", str);
        return EFAULT;
    }
    *str2 = '\0';

    const double tmp = strtod(str, NULL);
    if (errno == ERANGE && (tmp == -HUGE_VAL || tmp == HUGE_VAL))
    {
    	logger(LOG_INFO, "cpufrequency strtod failed str=%s", str);
    	return EFAULT;
    }

    s_cpufreq = tmp;
    if (s_cpufreq == 0)
    {
        s_cpufreq = 1; // set it to 1 to avoid div-zero errors just in case.
    	logger(LOG_INFO, "cpufrequency is zero! original value=%f", tmp);
    	return EFAULT;
    }

    logger(LOG_INFO, "cpufrequency is %f mhz => %lu", tmp, s_cpufreq);
    return 0;
#endif
}


} // namespace robo




