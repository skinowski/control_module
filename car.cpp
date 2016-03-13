/*
 * Copyright (C) 2016 Tolga Ceylan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 */
#include "car.h"
#include "logger.h"
#include "timer.h"
#include "sensor_state.h"
#include "utils.h"
#include <errno.h>
#include <math.h>
#include <assert.h>

// diameter of the rotary encoder shaft is 2.0 cm.
// with 1024 ticks for each rotation
#define WHEEL_CIRC         (2.0 * (M_PIl))
#define ENCODER_SCALER     (1024.0l / (WHEEL_CIRC) * 2.0l)

#define PID_KP             (0.45f)
#define PID_KI             (0.0f)
#define PID_KD             (9.0f)

#define SPEED_STEP (25)

namespace robo
{

int Car::convert_to_speed(const Pid &pid)
{
    const int speed = linear_map(
            static_cast<long double>(pid.output),
            -1024.0l, 1024.0l, 
            -255.0l, 255.0l
        );
    return speed;
}


Car::Car(MotorDriver *driver, SensorState *sensors)
    :
    m_driver(driver),
    m_sensors(sensors),
    m_left(driver),
    m_right(driver),
    m_leftSpeed(0),
    m_rightSpeed(0),
    m_previous(0),
    m_period(1)
{
    assert(driver);
    assert(sensors);
    reset_state();
}

Car::~Car()
{
    shutdown();
}

void Car::shutdown()
{
    logger(LOG_INFO, "car shutting down");
    m_right.shutdown();
    m_left.shutdown();
    m_driver->execute();
    reset_state();
}

void Car::reset_state()
{
    m_leftSpeed = 0;
    m_rightSpeed = 0;

    pid_init(m_leftPid, 0, PID_KP, PID_KI, PID_KD);
    pid_init(m_rightPid, 1, PID_KP, PID_KI, PID_KD);
}

int Car::initialize(int left_id, int right_id, uint64_t period)
{
    logger(LOG_INFO, "car initializing with period=%llu", period);
    reset_state();

    int res = 0;

    m_period = period;
    m_previous = 0;

    res = m_left.initialize(left_id);
    if (res)
        goto fail;

    res = m_right.initialize(right_id);
    if (res)
        goto fail;

    return 0;

fail:
    if (res)
           logger(LOG_ERROR, "cannot initialize car err=%d", res);
    return res;
}

int Car::run(int distance)
{
    const int ticks = distance * ENCODER_SCALER;
    logger(LOG_INFO, "car run distance=%d ticks=%d", distance, ticks);

    pid_init(m_leftPid, 0, PID_KP, PID_KI, PID_KD);
    pid_init(m_rightPid, 1, PID_KP, PID_KI, PID_KD);

    m_leftPid.target = ticks;
    m_rightPid.target = ticks;

    m_previous = 0;
    return 0;
}

void Car::stop()
{
    logger(LOG_INFO, "car stopping");

    m_left.set_speed(0);
    m_right.set_speed(0);

    m_driver->execute();

    reset_state();
}

int Car::turn(int angle)
{
    // this is BS, but let's start with something.
    const int ticks = angle * ENCODER_SCALER / 5.0f;

    logger(LOG_INFO, "car turn angle=%d ticks=%d", angle, ticks);

    pid_init(m_leftPid, 0, PID_KP, PID_KI, PID_KD);
    pid_init(m_rightPid, 1, PID_KP, PID_KI, PID_KD);

    m_leftPid.target = ticks > 0 ? ticks : -ticks;
    m_rightPid.target = ticks > 0 ? -ticks : ticks;

    m_previous = 0;
    return 0;
}

int Car::update_pid(uint64_t now, Pid &pid_left, Pid &pid_right)
{
    int res = 0;
    int res2 = 0;

    res = m_sensors->m_encoder.error;
    if (res)
        goto out;

    pid_left.actual  = -m_sensors->m_encoder.left;
    pid_right.actual = m_sensors->m_encoder.right;

    if (pid_left.target != 0)
        logger(LOG_INFO, "Car id=%d left=%u", pid_left.id, pid_left.actual);
    if (pid_right.target != 0)
        logger(LOG_INFO, "Car id=%d right=%u", pid_right.id, pid_right.actual);
    
    pid_update(pid_left, now);
    pid_update(pid_right, now);

    if (pid_left.target != 0)
        pid_logger(pid_left);
    if (pid_right.target != 0)
        pid_logger(pid_right);

    out:
    return res;
}

int Car::update_wheel(Wheel &wheel, int &speed, Pid &pid, bool &isExecute)
{
    int new_speed = pid.target ? convert_to_speed(pid) : 0;
    if (speed == new_speed)
        return 0;

    // ramp up slowly
    const int delta = new_speed - speed;
    if (delta > 0 && delta > SPEED_STEP)
        new_speed = speed + SPEED_STEP;

    logger(LOG_INFO, "Car pid=%d old_speed=%d new_speed=%d delta=%d", pid.id, speed, new_speed, delta);

    isExecute = true;
    speed = new_speed;

    int res = wheel.set_speed(speed);
    if (res)
        logger(LOG_ERROR, "Error in wheel set/release err=%d", res);

    return res;
}

int Car::update(uint64_t now)
{
    if (Timer::get_elapsed_usec(m_previous, now) <= m_period)
        return 0;
    m_previous = now;

    int res = 0;
    bool isExecute = false;

    res = update_pid(now, m_leftPid, m_rightPid);
    if (res) {
        logger(LOG_ERROR, "Error Car::update update_pid err=%d", res);
        goto fail;
    }

    res = update_wheel(m_left, m_leftSpeed, m_leftPid, isExecute);
    if (res) {
        logger(LOG_ERROR, "Error in Car::update update_wheel(left) err=%d", res);
        goto fail;
    }

    res = update_wheel(m_right, m_rightSpeed, m_rightPid, isExecute);
    if (res) {
        logger(LOG_ERROR, "Error in Car::update update_wheel(right) err=%d", res);
        goto fail;
    }

    if (isExecute) {
        res = m_driver->execute();
        if (res) {
            logger(LOG_ERROR, "Error in Car::update driver execute err=%d", res);
            goto fail;
        }
    }

fail:
    return res;
}

} // namespace robo



