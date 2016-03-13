/*
 * utils.h
 *
 */

#ifndef UTILS_H__
#define UTILS_H__

#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <algorithm>
#include <math.h>

#define HANDLE_EINTR(x) ({                   \
  decltype(x) _result;                         \
  do                                         \
    _result = (x);                           \
  while (_result == -1 && errno == EINTR);   \
  _result;                                   \
})

#define HANDLE_EAGAIN(x) ({                  \
  decltype(x) _result;                         \
  do                                         \
    _result = (x);                           \
  while (_result == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)); \
  _result;                                   \
})

#define HANDLE_EINTR_EAGAIN(x) ({            \
  decltype(x) _result;                         \
  do                                         \
    _result = (x);                           \
  while (_result == -1                       \
         && (errno == EINTR ||               \
             errno == EAGAIN ||              \
             errno == EWOULDBLOCK));         \
  _result;                                   \
})

namespace robo {

int build_sys_path(const char *partial_path, const char *prefix, char *full_path, size_t full_path_len);

int activate_cape_mgr_slot(const char *name);
int deactivate_cape_mgr_slot(const char *name);

uint64_t soundspeed_usec2cm(uint64_t usec, float temp);

int write_int_to_file(const char *filename, int value);
int write_buf_to_file(const char *filename, const char *buf, int bufLen);

int open_path(const char *dir1, const char *dir2, int &fd, int flags);
int write_buf_to_path(const char *dir1, const char *dir2, const char *buf, int bufLen);

template <typename T>
T clamp(T t1, T min, T max)
{
    return std::min(max, std::max(min, t1));
}

template <typename T>
T linear_map(T input, T input_min, T input_max, T output_min, T output_max)
{
    input = clamp(input, input_min, input_max);
    return input * (output_max - output_min) / (input_max - input_min);
}

template <typename T>
bool set_if_not_equal(T &dst, const T &src)
{
    if (dst == src)
        return false;

    dst = src;
    return true;
}

//
//    Calculates the altitude (in meters) from the specified atmospheric
//    pressure (in hPa), and sea-level pressure (in hPa).
//
//    @param  seaLevel      Sea-level pressure in hPa
//    @param  atmospheric   Atmospheric pressure in hPa
//
inline float pressure_to_altitude(float seaLevel, float atmospheric)
{
    // Equation taken from BMP180 datasheet (page 16):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    // Note that using the equation from wikipedia can give bad results
    // at high altitude.  See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

    return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

//    Calculates the pressure at sea level (in hPa) from the specified altitude 
//    (in meters), and atmospheric pressure (in hPa).  
//
//    @param  altitude      Altitude in meters
//    @param  atmospheric   Atmospheric pressure in hPa
inline float sea_level_for_altitude(float altitude, float atmospheric)
{
    // Equation taken from BMP180 datasheet (page 17):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    // Note that using the equation from wikipedia can give bad results
    // at high altitude.  See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

    return atmospheric / pow(1.0 - (altitude/44330.0), 5.255);
}

inline float to_degree(float radian)
{
    return radian * 180.0f / M_PI;
}

} // namespace robo


#endif /* UTILS_H__ */
