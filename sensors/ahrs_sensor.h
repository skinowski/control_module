
#ifndef AHRS_SENSOR_H__
#define AHRS_SENSOR_H__

#include <stdint.h>

namespace robo { namespace ahrs {

struct Sensor
{
    float         x;
    float         y;
    float         z;
    int         error;
    uint64_t    time;

    Sensor()
        :
        x(0.0f),
        y(0.0f),
        z(0.0f),
        error(0),
        time(0)
    {}

    void clear_data()
    {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        error = 0;
    }

    bool is_error() const
    {
        return error;
    }

    bool is_zero() const
    {
        return x == 0.0f && y == 0.0f && z == 0.0f;
    }

    bool is_good() const
    {
        return !is_error() && !is_zero();
    }
};

}} // namespace robo::arhs

#endif // AHRS_SENSOR_H__