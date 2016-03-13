/***************************************************************************
  This is a library for the LSM303 Accelerometer and magnentometer/compass
  Designed specifically to work with the Adafruit LSM303DLHC Breakout
  These displays use I2C to communicate, 2 pins are required to interface.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include "lsm303accel.h"

#include "logger.h"
#include "thread.h"
#include "i2c.h"
#include "timer.h"

#include <assert.h>

namespace robo {

const float LSM303ACCEL_MG_LSB = 0.001F;   // 1, 2, 4 or 12 mg per lsb
const float SENSORS_GRAVITY_EARTH = 9.80665F;

void LSM303Accel::s_initialize(void *arg, uint8_t *arg_blob)
{
    LSM303Accel *me = static_cast<LSM303Accel *>(arg);
    me->thr_initialize();
}

void LSM303Accel::s_measure(void *arg, uint8_t *arg_blob)
{
    LSM303Accel *me = static_cast<LSM303Accel *>(arg);
    me->thr_measure();
}

LSM303Accel::LSM303Accel(I2C *driver, Thread *thr)
    :
    m_driver(driver),
    m_thread(thr),
    m_period(0),
    m_initialized(false),
    m_error(0)
{
    assert(driver);
    assert(thr);
}

LSM303Accel::~LSM303Accel()
{
    shutdown();
}
 
int LSM303Accel::initialize(uint64_t period, Lsm303AccelFrequency frequency)
{
    logger(LOG_INFO, "LSM303Accel initializing period=%llu freq=%d", period, frequency);
    if (!m_thread->is_running() || m_initialized || !period)
        return EINVAL;

    m_initialized   = true;
    m_frequency     = frequency;
    m_period        = period;

    Thread::Cmd cmd;
    cmd.arg = this;
    cmd.fun = &LSM303Accel::s_initialize;

    int res = m_thread->queue_cmd(cmd);
    if (res)
        logger(LOG_INFO, "LSM303Accel initialize error err=%d err=%d", res, errno);
    return res;
}

void LSM303Accel::shutdown()
{
    logger(LOG_INFO, "LSM303Accel shutting down");
    if (!m_thread->is_running() || !m_initialized)
        return;

    // nothing to do really...
    m_initialized   = false;
    m_error         = 0;
}

int LSM303Accel::update(uint64_t now)
{
    if (!m_initialized)
        return 0;

    if (m_error)
        return m_error;

    return check_exec_cmd(now);
}

int LSM303Accel::check_exec_cmd(uint64_t now)
{
    assert(m_period);
    static uint64_t prev = now;

    // check elapsed since
    if (Timer::get_elapsed_usec(prev, now) <= m_period)
        return 0;

    prev = now;

    Thread::Cmd cmd;
    cmd.arg = this;
    cmd.fun = &LSM303Accel::s_measure;

    const int res = m_thread->queue_cmd(cmd);
    if (res)
        logger(LOG_INFO, "LSM303Accel check_exec_cmd error err=%d err=%d", res, errno);
    return res;
}

int LSM303Accel::get_reading(ahrs::Sensor &reading)
{
    if (m_error)
        return m_error;
    int ret = m_reading.get_value(reading);
    return ret;
}

void LSM303Accel::thr_initialize()
{
    const uint8_t freq = m_frequency;
    uint8_t id = 0;

    int res = 0;

    // initialize frequency
    res = m_driver->i2c_write16(res, LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, freq);

    // check if REG1_A has the set frequency (AKA connection check)
    res = m_driver->i2c_read8(res, LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, &id);

    if (!res && id != freq)
        res = EFAULT;

    if (res)
    {
        m_error = res;
        logger(LOG_ERROR, "LSM303Accel thr_initialize error err=%d", res);
    }
    else
    {
        logger(LOG_INFO, "LSM303Accel thr_initialize OK");
    }
}

void LSM303Accel::thr_measure()
{
    ahrs::Sensor reading;

    int len = 0;
    uint8_t data[6] = { LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80 };

    reading.error = m_driver->i2c_write8(reading.error, LSM303_ADDRESS_ACCEL, data[0]);
    reading.error = m_driver->i2c_read(reading.error, data, sizeof(data), len);

    if (!reading.error && len != sizeof(data))
        reading.error = EIO;

    if (!reading.error)
    {
        uint8_t xlo = data[0];
        uint8_t xhi = data[1];
        uint8_t ylo = data[2];
        uint8_t yhi = data[3];
        uint8_t zlo = data[4];
        uint8_t zhi = data[5];

        // Shift values to create properly formed integer (low byte first)
        reading.x = (int16_t)(xlo | (xhi << 8)) >> 4;
        reading.y = (int16_t)(ylo | (yhi << 8)) >> 4;
        reading.z = (int16_t)(zlo | (zhi << 8)) >> 4;

        reading.x *= LSM303ACCEL_MG_LSB * SENSORS_GRAVITY_EARTH;
        reading.y *= LSM303ACCEL_MG_LSB * SENSORS_GRAVITY_EARTH;
        reading.z *= LSM303ACCEL_MG_LSB * SENSORS_GRAVITY_EARTH;
    }

    reading.time = Timer().get_now();
    int res = m_reading.set_value(reading);
    if (res)
        m_error = res;
}


} // namespace robo
