#include "lsm303mag.h"

#include "logger.h"
#include "thread.h"
#include "i2c.h"
#include "timer.h"

#include <assert.h>

namespace robo {

const float SENSORS_GAUSS_TO_MICROTESLA = 100.0f;   /**< Gauss to micro-Tesla multiplier */

void LSM303Mag::s_initialize(void *arg, uint8_t *arg_blob)
{
    LSM303Mag *me = static_cast<LSM303Mag *>(arg);
    me->thr_initialize(arg_blob[0]);
}

void LSM303Mag::s_measure(void *arg, uint8_t *arg_blob)
{
    LSM303Mag *me = static_cast<LSM303Mag *>(arg);
    me->thr_measure();
}

LSM303Mag::LSM303Mag(I2C *driver, Thread *thr)
    :
    m_driver(driver),
    m_thread(thr),
    m_period(0),
    m_initialized(false),
    m_autoRange(false),
    m_error(0),
    m_gain(LSM303_MAGGAIN_1_3),
    m_gauss_lsb_xy(1100.0F),  // Varies with gain
    m_gauss_lsb_z(980.0F)   // Varies with gain
{
    assert(driver);
    assert(thr);
}

LSM303Mag::~LSM303Mag()
{
    shutdown();
}
 
int LSM303Mag::initialize(uint64_t period, Lsm303MagGain gain, bool enableAutoRange)
{
    logger(LOG_INFO, "LSM303Mag initializing");
    if (!m_thread->is_running() || m_initialized || !period)
        return EINVAL;

    m_initialized   = true;
    m_period        = period;
    m_autoRange     = enableAutoRange;

    Thread::Cmd cmd;
    cmd.arg = this;
    cmd.fun = &LSM303Mag::s_initialize;
    cmd.arg_blob[0] = gain;

    int res = m_thread->queue_cmd(cmd);
    if (res)
        logger(LOG_INFO, "LSM303Mag initialize error err=%d err=%d", res, errno);
    return res;
}

void LSM303Mag::shutdown()
{
    logger(LOG_INFO, "LSM303Mag shutting down");
    if (!m_thread->is_running() || !m_initialized)
        return;

    // nothing to do really...
    m_initialized   = false;
    m_error         = 0;
}

int LSM303Mag::update(uint64_t now)
{
    if (!m_initialized)
        return 0;

    if (m_error)
        return m_error;

    return check_exec_cmd(now);
}

int LSM303Mag::check_exec_cmd(uint64_t now)
{
    assert(m_period);
    static uint64_t prev = now;

    // check elapsed since
    if (Timer::get_elapsed_usec(prev, now) <= m_period)
        return 0;

    prev = now;

    Thread::Cmd cmd;
    cmd.arg = this;
    cmd.fun = &LSM303Mag::s_measure;

    int res = m_thread->queue_cmd(cmd);
    if (res)
        logger(LOG_INFO, "LSM303Mag check_exec_cmd error err=%d err=%d", res, errno);
    return res;
}

int LSM303Mag::get_reading(ahrs::Sensor &reading)
{
    if (m_error)
        return m_error;
    int ret = m_reading.get_value(reading);
    return ret;
}

void LSM303Mag::thr_initialize(uint8_t arg)
{
    int res = 0;
    uint8_t id = 0;

    // Enable the magnetometer
    res = m_driver->i2c_write16(res, LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_MR_REG_M, 0x00);

    // Check if CRA_REG_M has the default value (AKA connection check)
    res = m_driver->i2c_read8(res, LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRA_REG_M, &id);

    if (!res && id != 0x10)
        res = EFAULT;

    // Set the gain to a known level
    m_gain = static_cast<Lsm303MagGain>(arg);
    if (!res)
        res = thr_set_mag_gain(arg);
    
    if (res)
    {
        m_error = res;
        logger(LOG_ERROR, "LSM303Mag thr_initialize error err=%d", res);
    }
    else
    {
        logger(LOG_INFO, "LSM303Mag thr_initialize OK");
    }
}

void LSM303Mag::thr_read(ahrs::Sensor &reading)
{
    int len = 0;
    uint8_t data[6] = { LSM303_REGISTER_MAG_OUT_X_H_M };

    reading.error = m_driver->i2c_write8(reading.error, LSM303_ADDRESS_MAG, data[0]);
    reading.error = m_driver->i2c_read(reading.error, data, sizeof(data), len);

    if (!reading.error && len != sizeof(data))
        reading.error = EIO;

    if (reading.error)
        return;

    uint8_t xlo = data[1];
    uint8_t xhi = data[0];
    uint8_t ylo = data[3];
    uint8_t yhi = data[2];
    uint8_t zlo = data[5];
    uint8_t zhi = data[4];

    // Shift values to create properly formed integer (low byte first)
    reading.x = (int16_t)(xlo | ((int16_t)xhi << 8));
    reading.y = (int16_t)(ylo | ((int16_t)yhi << 8));
    reading.z = (int16_t)(zlo | ((int16_t)zhi << 8));
}

void LSM303Mag::thr_measure()
{
    ahrs::Sensor reading;

    const bool isAutoRange  = m_autoRange;

    while (!reading.error)
    {
        thr_read(reading);
        if (reading.error)
            break;

        if (!isAutoRange)
            break;
    
        /* Check if the sensor is saturating or not */
        if (reading.x < 2040 && reading.x > -2040 &&
            reading.y < 2040 && reading.y > -2040 &&
            reading.z < 2040 && reading.z > -2040)
            break;

        /* Saturating .... increase the range if we can */
        Lsm303MagGain previous = m_gain;
        switch(m_gain)
        {
            case LSM303_MAGGAIN_5_6:  m_gain = LSM303_MAGGAIN_8_1; break;
            case LSM303_MAGGAIN_4_7:  m_gain = LSM303_MAGGAIN_5_6; break;
            case LSM303_MAGGAIN_4_0:  m_gain = LSM303_MAGGAIN_4_7; break;
            case LSM303_MAGGAIN_2_5:  m_gain = LSM303_MAGGAIN_4_0; break;
            case LSM303_MAGGAIN_1_9:  m_gain = LSM303_MAGGAIN_2_5; break;
            case LSM303_MAGGAIN_1_3:  m_gain = LSM303_MAGGAIN_1_9; break;
            default: break;
        }

        if (previous == m_gain)
            break;

        reading.error = thr_set_mag_gain(m_gain);
    }
  
    reading.x = reading.x / m_gauss_lsb_xy * SENSORS_GAUSS_TO_MICROTESLA;
    reading.y = reading.y / m_gauss_lsb_xy * SENSORS_GAUSS_TO_MICROTESLA;
    reading.z = reading.z / m_gauss_lsb_z  * SENSORS_GAUSS_TO_MICROTESLA;

    reading.time = Timer().get_now();
    int res = m_reading.set_value(reading);
    if (res)
        m_error = res;
}

int LSM303Mag::thr_set_mag_gain(uint8_t gain)
{
  int res = 0;
  res = m_driver->i2c_write16(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, gain);
  if (res)
    return res;

  switch(gain)
  {
    case LSM303_MAGGAIN_1_3:
      m_gauss_lsb_xy = 1100;
      m_gauss_lsb_z  = 980;
      break;
    case LSM303_MAGGAIN_1_9:
      m_gauss_lsb_xy = 855;
      m_gauss_lsb_z  = 760;
      break;
    case LSM303_MAGGAIN_2_5:
      m_gauss_lsb_xy = 670;
      m_gauss_lsb_z  = 600;
      break;
    case LSM303_MAGGAIN_4_0:
      m_gauss_lsb_xy = 450;
      m_gauss_lsb_z  = 400;
      break;
    case LSM303_MAGGAIN_4_7:
      m_gauss_lsb_xy = 400;
      m_gauss_lsb_z  = 355;
      break;
    case LSM303_MAGGAIN_5_6:
      m_gauss_lsb_xy = 330;
      m_gauss_lsb_z  = 295;
      break;
    case LSM303_MAGGAIN_8_1:
      m_gauss_lsb_xy = 230;
      m_gauss_lsb_z  = 205;
      break;
  } 
  return 0;
}

} // namespace robo
