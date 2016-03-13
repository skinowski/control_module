/*
 * Copyright (C) 2016 Tolga Ceylan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 */
#include "robo.h"
#include "logger.h"
#include "scheduler.h"
#include "utils.h"

#include <algorithm>
#include <errno.h>

#define LEFT_WHEEL_ID 1
#define RIGHT_WHEEL_ID 2

#define I2C_BUS1 1

#define MOTORSHIELD_ADDRESS (0x60)
#define MOTORSHIELD_FREQ (1600)

#define SCANNER_INITIAL_POSITION 90
#define SCANNER_ANGLE_STEP 5

#define THR0_PRIORITY 10
#define THR1_PRIORITY 20
#define THR2_PRIORITY 19
#define THR0_POLICY SCHED_FIFO
#define THR1_POLICY SCHED_FIFO
#define THR2_POLICY SCHED_FIFO

// usec values
#define BMP085_PERIOD             (2000000)     // every 2 secs
#define L3G20_PERIOD              (20000)       // every 20 msec
#define LSM303ACCEL_PERIOD         (20000)     // every 20 msec
#define LSM303MAG_PERIOD         (20000)     // every 20 msec
#define ENCODER_PERIOD          (20000)        // every 20 msec

#define CAR_PERIOD                (20000)        // every 20 msec

#define AHRS_SAMPLE_FREQUENCY    (50)        // 1000 / 20 msec

#define POLLER_MAX_WAIT         (2000)        // 2 msec

#define ROBOT_WARMUP_DELAY        (2000000)    // 2 secs

namespace robo
{

Stats::Stats()
{
    reset();
}

void Stats::reset()
{
    m_maxFrameTime = 0;
    m_minFrameTime = std::numeric_limits<uint64_t>::max();
    m_avgFrameTime = 0;
    m_numOfFrames = 0;
    m_avgFrameSmoother = 16;
    m_displayPeriod = 1000000 * 1; // 1 sec
    m_totalFrameTime = 0;
}

void Stats::print() const
{
    logger(LOG_INFO, "Stats maxFrameTime=%llu minFrameTime=%llu avgFrameTime=%llu",
            m_maxFrameTime,
            m_minFrameTime,
            m_avgFrameTime
    );
    logger(LOG_INFO, "Stats numOfFrames=%llu avgFrameSmoother=%llu displayPeriod=%llu totalFrameTime=%llu",
            m_numOfFrames,
            m_avgFrameSmoother,
            m_displayPeriod,
            m_totalFrameTime
    );
}

bool Stats::update_stats(uint64_t frameTime)
{
    m_maxFrameTime = std::max(m_maxFrameTime, frameTime);
    m_minFrameTime = std::min(m_minFrameTime, frameTime);
    m_totalFrameTime += frameTime;

    if (m_avgFrameTime == 0)
    {
        m_avgFrameTime = frameTime;
    }
    else
    {
        if (frameTime < m_avgFrameTime)
            m_avgFrameTime -= (m_avgFrameTime - frameTime) / m_avgFrameSmoother;
        else
            m_avgFrameTime += (frameTime - m_avgFrameTime) / m_avgFrameSmoother;
    }

    ++m_numOfFrames;

    static uint64_t elapsed = 0;
    elapsed += frameTime;
    if (elapsed >= m_displayPeriod)
    {
        elapsed = 0;
        return true;
    }
    return false;
}

State::State()
{
    reset();
}

void State::start()
{
    isTriggerAdvance = true;
}

void State::reset(uint64_t start_deadline)
{
    isTriggerAdvance = false;

    distance = 10;
    maxRun = 1000000 * 5; // x seconds max
    minFrameTime = 1000; // 10 msec loop delay  
    isDone = false; // terminate flag

    warmupDeadline = start_deadline;
}

void State::print() const
{
    logger(LOG_INFO, "State "
            "isTriggerAdvance=%d",
            isTriggerAdvance
    );

    logger(LOG_INFO, "State distance=%d maxRun=%llu minFrameTime=%llu isDone=%d",
            distance,
            maxRun,
            minFrameTime,
            isDone
    );
}

Robot::Robot(int demo_num)
    :
    m_demo_num(demo_num),
    m_timer(),
    m_poller(),
    m_thread1(),
    m_i2c1(),
    m_driver(&m_i2c1, &m_thread1),
    m_car(&m_driver, &m_sensor_state),
    m_bmp085(&m_i2c1, &m_thread1),
    m_l3gd20(&m_i2c1, &m_thread1),
    m_lsm303accel(&m_i2c1, &m_thread1),
    m_lsm303mag(&m_i2c1, &m_thread1),
    m_encoder(&m_i2c1, &m_thread1),
    m_stats(),
    m_sensor_state(AHRS_SAMPLE_FREQUENCY),
    m_state()
{
    logger(LOG_INFO, "Robot demo=%d", demo_num);
}

Robot::~Robot()
{
    shutdown();
}

void Robot::shutdown()
{
    logger(LOG_INFO, "Robot shutting down");
    m_state.print();

    m_thread1.shutdown();

    m_bmp085.shutdown();
    m_l3gd20.shutdown();
    m_lsm303accel.shutdown();
    m_lsm303mag.shutdown();
    m_encoder.shutdown();

    m_car.shutdown();
    m_driver.shutdown();
    m_i2c1.shutdown();

    m_poller.shutdown();
    m_state.reset();
}

int Robot::update_io_thread(uint64_t now)
{
    int ret = m_thread1.check_signal_io();
    return ret;
}

int Robot::update_encoder(uint64_t now)
{
    int res = m_encoder.update(now);
    if (res)
        return res;

    if (!m_encoder.is_initialized())
        return 0;

    res = m_encoder.get_reading(m_sensor_state.m_encoder);
    return res;
}

int Robot::update_lsm303accel(uint64_t now)
{
    int res = m_lsm303accel.update(now);
    if (res)
        return res;

    if (!m_lsm303accel.is_initialized())
        return 0;

    res = m_lsm303accel.get_reading(m_sensor_state.m_accel);
    return res;
}

int Robot::update_lsm303mag(uint64_t now)
{
    int res = m_lsm303mag.update(now);
    if (res)
        return res;

    if (!m_lsm303mag.is_initialized())
        return 0;

    res = m_lsm303mag.get_reading(m_sensor_state.m_mag);
    return res;
}

int Robot::update_l3gd20(uint64_t now)
{
    int res = m_l3gd20.update(now);
    if (res)
        return res;

    if (!m_l3gd20.is_initialized())
        return 0;

    res = m_l3gd20.get_reading(m_sensor_state.m_gyro);
    return res;
}

int Robot::update_bmp085(uint64_t now)
{
    int res = m_bmp085.update(now);
    if (res)
        return res;

    if (!m_bmp085.is_initialized())
        return 0;
    
    int res_temp = m_bmp085.get_reading(BMP085_TYPE_TEMPERATURE, m_sensor_state.m_temperature);
    int res_pres = m_bmp085.get_reading(BMP085_TYPE_PRESSURE, m_sensor_state.m_pressure);
    return res_temp ? res_temp : res_pres;
}

int Robot::update_car(uint64_t now)
{
    int ret = m_car.update(now);
    if (ret)
        return ret;

    if (!m_driver.is_initialized() || !m_state.isTriggerAdvance)
        return 0;

    logger(LOG_INFO,"triggering car to move dist=%d", m_state.distance);

    ret = m_car.run(m_state.distance);
//    ret = m_car.turn(45);
    if (ret)
        return ret;

    m_state.isTriggerAdvance = false;
    return ret;
}

int Robot::update_driver(uint64_t now)
{
    return m_driver.update(now);
}

int Robot::update_sensor_state(uint64_t now)
{
    // TODO: Here tell the sensors if we are moving or not.
    // If not moving, we use the samples as reference. (eg. determine z 
    // earth gravity in accelerometer)
    return m_sensor_state.update_sensor_state(now);
}

int Robot::update_poller(uint64_t start, uint64_t &elapsed)
{
    int ret = 0;

    elapsed = Timer::get_elapsed_usec(start, m_timer.get_now());
    ret = m_poller.update(POLLER_MAX_WAIT);
    elapsed = Timer::get_elapsed_usec(start, m_timer.get_now());

    return ret;
}

int Robot::update()
{
    int ret = 0;

    uint64_t         now     = m_timer.get_now();
    uint64_t         elapsed = 0;
    const uint64_t     start     = now;

    // Let's start with strict error handling first. Going
    // forward we might want to still run some of these even
    // if certain components fail.
    ret = update_sensor_state(now);
    if (ret) {
        logger(LOG_ERROR, "Robot update_sensor_state err=%d %s", ret, strerror(ret));
        goto done;
    }
    ret = update_bmp085(now);
    if (ret) {
        logger(LOG_ERROR, "Robot update_bmp085 err=%d %s", ret, strerror(ret));
        goto done;
    }
    ret = update_lsm303accel(now);
    if (ret) {
        logger(LOG_ERROR, "Robot update_lsm303accel err=%d %s", ret, strerror(ret));
        goto done;
    }
    ret = update_lsm303mag(now);
    if (ret) {
        logger(LOG_ERROR, "Robot update_lsm303mag err=%d %s", ret, strerror(ret));
        goto done;
    }
    ret = update_l3gd20(now);
    if (ret) {
        logger(LOG_ERROR, "Robot update_l3gd20 err=%d %s", ret, strerror(ret));
        goto done;
    }
    ret = update_encoder(now);
    if (ret) {
        logger(LOG_ERROR, "Robot update_encoder err=%d %s", ret, strerror(ret));
        goto done;
    }
    ret = update_car(now);
    if (ret) {
        logger(LOG_ERROR, "Robot update_car err=%d %s", ret, strerror(ret));
        goto done;
    }
    ret = update_driver(now);
    if (ret) {
        logger(LOG_ERROR, "Robot update_driver err=%d %s", ret, strerror(ret));
        goto done;
    }
    ret = update_io_thread(now);
    if (ret) {
        logger(LOG_ERROR, "Robot update_io_thread err=%d %s", ret, strerror(ret));
        goto done;
    }
    ret = update_poller(start, elapsed);
    if (ret) {
        logger(LOG_ERROR, "Robot update_poller err=%d %s", ret, strerror(ret));
        goto done;
    }

done:
    if (m_stats.update_stats(elapsed))
        m_stats.print();

    if (!ret && (m_state.isDone || (m_state.maxRun > 0 && m_state.maxRun <= m_stats.m_totalFrameTime)))
        ret = ETIME;

    if (ret)
    {
        logger(LOG_ERROR, "Robot exit err=%d %s", ret, strerror(ret));
        error_shut_sequence();
    }

    if (m_state.warmupDeadline && m_state.warmupDeadline <= now)
    {
        m_state.warmupDeadline = 0;
        m_state.start();
    }

    return ret;
}

void Robot::error_shut_sequence()
{
    m_car.stop();
    m_thread1.check_signal_io();
}

int Robot::initialize()
{
    logger(LOG_INFO, "Robot initializing");

    int ret = 0;

    ret = set_scheduler(THR0_POLICY, THR0_PRIORITY);
    if (ret) {
        logger(LOG_ERROR, "Robot set_scheduler err=%d %s", ret, strerror(ret));
        goto done;
    }

    ret = Timer::initialize();
    if (ret) {
        logger(LOG_ERROR, "Robot Timer::initialize err=%d %s", ret, strerror(ret));
        goto done;
    }

    ret = m_poller.initialize();
    if (ret) {
        logger(LOG_ERROR, "Robot Poller::initialize err=%d %s", ret, strerror(ret));
        goto done;
    }

    ret = m_thread1.initialize(THR1_PRIORITY, THR1_POLICY);
    if (ret) {
        logger(LOG_ERROR, "Robot Thread::initialize err=%d %s", ret, strerror(ret));
        goto done;
    }

    ret = m_i2c1.initialize(I2C_BUS1);
    if (ret) {
        logger(LOG_ERROR, "Robot I2C::initialize err=%d %s", ret, strerror(ret));
        goto done;
    }

    ret = m_driver.initialize(MOTORSHIELD_ADDRESS, MOTORSHIELD_FREQ);
    if (ret) {
        logger(LOG_ERROR, "Robot MotorDriver::initialize err=%d %s", ret, strerror(ret));
        goto done;
    }

    ret = m_car.initialize(LEFT_WHEEL_ID, RIGHT_WHEEL_ID, CAR_PERIOD);
    if (ret) {
        logger(LOG_ERROR, "Robot Car::initialize err=%d %s", ret, strerror(ret));
        goto done;
    }

    ret = m_bmp085.initialize(BMP085_PERIOD);
    if (ret) {
        logger(LOG_ERROR, "Robot BMP085::initialize err=%d %s", ret, strerror(ret));
        goto done;
    }

    ret = m_l3gd20.initialize(L3G20_PERIOD, GYRO_RANGE_250DPS, false);
    if (ret) {
        logger(LOG_ERROR, "Robot L3GD20::initialize err=%d %s", ret, strerror(ret));
        goto done;
    }

    ret = m_lsm303accel.initialize(LSM303ACCEL_PERIOD, LSM303_ACCEL_400HZ);
    if (ret) {
        logger(LOG_ERROR, "Robot LSM303Accel::initialize err=%d %s", ret, strerror(ret));
        goto done;
    }

    ret = m_lsm303mag.initialize(LSM303MAG_PERIOD, LSM303_MAGGAIN_1_3, false);
    if (ret) {
        logger(LOG_ERROR, "Robot LSM303Mag::initialize err=%d %s", ret, strerror(ret));
        goto done;
    }

    ret = m_encoder.initialize(ENCODER_PERIOD);
    if (ret) {
        logger(LOG_ERROR, "Robot Encoder::initialize err=%d %s", ret, strerror(ret));
        goto done;
    }

    m_state.reset(m_timer.get_now() + ROBOT_WARMUP_DELAY);

done:
    return ret;
}

} // namespace robo
