/*
 * Copyright (C) 2016 Tolga Ceylan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 */
#include "sensor_state.h"

#include "timer.h"
#include "logger.h"
#include "utils.h"

namespace robo {

SensorState::SensorState(float sample_frequency)
    :
    m_accel_last(0),
    m_mag_last(0),
    m_gyro_last(0),
    m_temperature_last(0),
    m_pressure_last(0)
{
    m_state.sample_frequency = sample_frequency;
}    

float SensorState::get_temperature()
{
    // simply return last reading if we have any.
    // Temperature is not that critical except in
    // sonar/distance calculations.
    if (!m_temperature.time)
        return 20.0f;
    return m_temperature.value;
}

void SensorState::bmp_log(uint64_t last, const BMP085::Reading &sensor, const char *prefix)
{
    if (last != sensor.time)
        logger(LOG_INFO,"%s val=%f time=%llu", prefix, sensor.value, sensor.time);
}

void SensorState::ahrs_log(uint64_t last, const ahrs::Sensor &sensor, const char *prefix)
{
    if (last != sensor.time)
    {
        logger(LOG_INFO,"%s x:y:z=%f:%f:%f err=%d time=%llu",
            prefix,
            sensor.x,
            sensor.y,
            sensor.z,
            sensor.error,
            sensor.time
        );
    }
    if (sensor.is_error())
        logger(LOG_ERROR, "%s error=%d", prefix, sensor.error);
}

void SensorState::ahrs_log_state(uint64_t now)
{
    logger(LOG_INFO, "SensorState twoKp=%f twoKi=%f integralFBx=%f integralFBy=%f integralFBz=%f",
        m_state.twoKp,
        m_state.twoKi,
        m_state.integralFBx,
        m_state.integralFBy,
        m_state.integralFBz
    );
    logger(LOG_INFO, "SensorState orientation=%f:%f:%f:%f",
        m_state.orient.q0,
        m_state.orient.q1,
        m_state.orient.q2,
        m_state.orient.q3
    );

    // For logging purposes, let's spit out Euler angles:

    const ahrs::Quaternion &q = m_state.orient;

    const float x = atan2(2 * (q.q0 + q.q1), 1 - 2 * (q.q1 * q.q1 + q.q2 * q.q2));
    const float y = asin(2 * (q.q0 * q.q2 - q.q3 * q.q1));
    const float z = atan2(2 * (q.q0 * q.q3 + q.q1 * q.q2), 1 - 2 * (q.q2 * q.q2 + q.q3 * q.q3));

    logger(LOG_INFO, "SensorState Euler=%f:%f:%f", to_degree(x), to_degree(y), to_degree(z));
}


int SensorState::update_sensor_state(uint64_t now)
{
    ahrs_log(m_accel_last, m_accel, "accelerator");
    ahrs_log(m_mag_last, m_mag, "magnetometer");
    ahrs_log(m_gyro_last, m_gyro, "gyro");
    bmp_log(m_temperature_last, m_temperature, "temperature");
    bmp_log(m_pressure_last, m_pressure, "pressure");

    // Clear stale data here. This is to prevent mahony_update()
    // to accidentally reintegrate old data.

    if (m_accel_last == m_accel.time)
        m_accel.clear_data();

    if (m_mag_last == m_mag.time)
        m_mag.clear_data();

    if (m_gyro_last == m_gyro.time)
        m_gyro.clear_data();

    if (mahony_update(m_state, m_gyro, m_accel, m_mag))
        ahrs_log_state(now);
    
    m_accel_last = m_accel.time;
    m_gyro_last = m_gyro.time;
    m_mag_last = m_mag.time;
    m_temperature_last = m_temperature.time;
    m_pressure_last = m_pressure.time;

    return 0;
}

} // namespace robo