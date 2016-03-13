/*
 * Copyright (C) 2016 Tolga Ceylan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 */
#ifndef SENSOR_STATE_H_
#define SENSOR_STATE_H_

// sensors
#include "bmp085.h"
#include "l3gd20.h"
#include "lsm303accel.h"
#include "lsm303mag.h"
#include "encoder.h"
#include "ahrs_mahony.h"

#include <stdint.h>

namespace robo {

struct SensorState
{
    SensorState(float sample_frequency);

    int update_sensor_state(uint64_t now);

    void bmp_log(uint64_t last, const BMP085::Reading &sensor, const char *prefix);
    void ahrs_log(uint64_t last, const ahrs::Sensor &sensor, const char *prefix);

    void ahrs_log_state(uint64_t now);

    float get_temperature();

    ahrs::State            m_state;

    ahrs::Sensor         m_accel;
    ahrs::Sensor         m_mag;
    ahrs::Sensor         m_gyro;

    BMP085::Reading     m_temperature;
    BMP085::Reading     m_pressure;

    Encoder::Reading    m_encoder;

    uint64_t             m_accel_last;
    uint64_t             m_mag_last;
    uint64_t             m_gyro_last;
    uint64_t             m_temperature_last;
    uint64_t             m_pressure_last;
};


} // namespace robo

#endif /* SENSOR_STATE_H_ */
