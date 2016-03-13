
/***************************************************************************
  This is a library for the BMP085 pressure sensor
  Designed specifically to work with the Adafruit BMP085 or BMP180 Breakout 
  ----> http://www.adafruit.com/products/391
  ----> http://www.adafruit.com/products/1603
 
  These displays use I2C to communicate, 2 pins are required to interface.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include "bmp085.h"

#include <math.h>
#include <limits.h>
#include <string.h>

#include "logger.h"
#include "thread.h"
#include "i2c.h"
#include "timer.h"
#include <assert.h>

// I2C ADDRESS/BITS
#define BMP085_ADDRESS    (0x77)

namespace robo {

void BMP085::s_initialize(void *arg, uint8_t *arg_blob)
{
    BMP085 *me = static_cast<BMP085 *>(arg);
    me->thr_initialize();
}

void BMP085::s_measure(void *arg, uint8_t *arg_blob)
{
    BMP085 *me = static_cast<BMP085 *>(arg);
    me->thr_measure(arg_blob[0]);
}


BMP085::Reading::Reading()
    :
    value(0.0f),
    time(0)
{
}

BMP085::BMP085(I2C *driver, Thread *thr)
    :
    m_driver(driver),
    m_thread(thr),
    m_mode(BMP085_MODE_ULTRAHIGHRES),
    m_initialized(false),
    m_state(0),
    m_b5(0),
    m_error(0)
{
    memset(&m_coeffs, 0, sizeof(m_coeffs));

    m_readings[BMP085_TYPE_PRESSURE]    = MutexVal<Reading>();
    m_readings[BMP085_TYPE_TEMPERATURE] = MutexVal<Reading>();

    m_state_wait[0] = 0;
    m_state_wait[1] = 5000;
    m_state_wait[2] = get_pressure_wait(m_mode);
}

BMP085::~BMP085()
{
    shutdown();
}
 
int BMP085::initialize(uint64_t period)
{
    logger(LOG_INFO, "BMP085 initializing");
    if (!m_thread->is_running() || m_initialized || !period)
        return EINVAL;

    m_initialized = true;
    m_state_wait[0] = period;

    Thread::Cmd cmd;
    cmd.arg = this;
    cmd.fun = &BMP085::s_initialize;

    int res = m_thread->queue_cmd(cmd);
    if (res)
        logger(LOG_INFO, "BMP085 initialize error err=%d err=%d", res, errno);
    return res;
}

void BMP085::shutdown()
{
    logger(LOG_INFO, "BMP085 shutting down");
    if (!m_thread->is_running() || !m_initialized)
        return;

    m_initialized = false;
    m_error = 0;
}

void BMP085::thr_initialize()
{
    int len = 0;
    uint8_t data = BMP085_REGISTER_CHIPID;
    int res = 0;

    res = m_driver->i2c_set_slave_address(BMP085_ADDRESS);
    res = m_driver->i2c_write(res, &data, sizeof(data));
    res = m_driver->i2c_read(res, &data, sizeof(data), len);

    if (!res && (data != 0x55 || len != sizeof(data)))
        res = EFAULT;

    if (!res)
        res = thr_read_coeff();

    // start the sampling loop
    m_state = 1;

    if (res)
    {
        logger(LOG_ERROR, "BMP085 thr_initialize error err=%d", res);
        m_error = res;
    }
    else
    {
        logger(LOG_INFO, "BMP085 thr_initialize OK");
    }
}

int BMP085::update(uint64_t now)
{
    if (!m_initialized)
        return 0;

    if (m_error)
        return m_error;

    if (!m_state)
        return 0;

    return check_exec_cmd(now);
}

int BMP085::check_exec_cmd(uint64_t now)
{
    assert(m_state);

    // This approach is thread safe. Why? Because the I/O thread sets a state, then
    // we detect that state change here in main thread. And our command sent from here
    // can only make I/O thread change that state again. 

    static uint64_t prev_time[3] =  { 0, 0, 0 };
    static int      is_done[3] =    { 0, 0, 0 };

    // read m_state once, just to be safe
    const size_t state = m_state;
    const size_t idx = state - 1;

    // do not execute same command more than once.. Remember update() is calling us
    // every frame.
    if (is_done[idx])
        return 0;

    // initialize the prev time
    if (prev_time[idx] == 0)
        prev_time[idx] = now;

    // check elapsed since
    if (Timer::get_elapsed_usec(prev_time[idx], now) <= m_state_wait[idx])
        return 0;

    // reset everything.
    memset(prev_time, 0, sizeof(prev_time));
    memset(is_done, 0, sizeof(is_done));

    // mark this state as done, so that we don't re-enter here in every frame
    is_done[idx] = 1;

    logger(LOG_INFO, "BMP085 check_exec_cmd prev_time=%llu:%llu:%llu is_done=%d:%d:%d state=%d wait=%llu", 
        prev_time[0],
        prev_time[1],
        prev_time[2],
        is_done[0],
        is_done[1],
        is_done[2],
        state,
        m_state_wait[idx]
    );

    Thread::Cmd cmd;

    cmd.arg         = this;
    cmd.fun         = &BMP085::s_measure;
    cmd.arg_blob[0] = state;

    return m_thread->queue_cmd(cmd);
}

int BMP085::thr_read_temperature(float &temp, int32_t &b5)
{
    uint16_t reading = 0;
    int res = m_driver->i2c_read16(BMP085_ADDRESS, BMP085_REGISTER_TEMPDATA, &reading);

    if (!res)
    {
        // Calculations from AdaFruit code
        b5 = thr_compute_b5(reading);
        float t = (m_b5 + 8) >> 4;
        t /= 10;
        temp = t;
    }
    return res;
}

int BMP085::thr_read_pressure(float &pressure)
{
    uint8_t  p8 = 0;
    uint16_t p16 = 0;

    int res = 0;

    res = m_driver->i2c_read16(res, BMP085_ADDRESS, BMP085_REGISTER_PRESSUREDATA, &p16);
    res = m_driver->i2c_read8(res, BMP085_ADDRESS, BMP085_REGISTER_PRESSUREDATA + 2, &p8);

    if (!res)
    {
        // Calculations from AdaFruit code
        int32_t p32 = (uint32_t)p16 << 8;
        p32 += p8;
        p32 >>= (8 - m_mode);
        pressure = thr_get_pressure(p32);
    }

    return res;
}

int BMP085::thr_request_temperature()
{
    return m_driver->i2c_write16(
        BMP085_ADDRESS, 
        BMP085_REGISTER_CONTROL, 
        BMP085_REGISTER_READTEMPCMD
    );
}
int BMP085::thr_request_pressure()
{
    return m_driver->i2c_write16(
        BMP085_ADDRESS, 
        BMP085_REGISTER_CONTROL, 
        BMP085_REGISTER_READPRESSURECMD + (m_mode << 6)
    );
}

int BMP085::thr_set_metric(Bmp085ReadingType type, float value)
{
    BMP085::Reading reading;

    reading.value = value;
    reading.time = Timer().get_now();

    if (type >= BMP085_TYPE_MAX || type < 0)
        return EINVAL;
    return m_readings[type].set_value(reading);
}

void BMP085::thr_measure(uint8_t state)
{
    int res = 0;

    if (state == 1)
    {
        res = thr_request_temperature();
        m_state = 2;
    }
    else if (state == 2)
    {
        float temperature = 0.0f;
        res = thr_read_temperature(temperature, m_b5);
        if (!res)
            res = thr_set_metric(BMP085_TYPE_TEMPERATURE, temperature);
        if (!res)
            res = thr_request_pressure();
        m_state = 3;
    }
    else if (state == 3)
    {
        float pressure = 0.0f;
        res = thr_read_pressure(pressure);
        if (!res)
            res = thr_set_metric(BMP085_TYPE_PRESSURE, pressure);
        m_state = 1; // switch back to state 1 (LOOP)
    }

    logger(LOG_INFO, "BMP085 thr_measure command=%u state=%d", state, m_state); 

    // With this approach, if an error occurs, we stop processing everything...
    // We could re-try if this is desired in the future.
    if (res)
    {
        m_state = 0;
        m_error = res;
        logger(LOG_ERROR, "BMP085 thr_measure error err=%d", res);
    }
}

uint64_t BMP085::get_pressure_wait(uint8_t mode) const
{
    switch(mode)
    {
      case BMP085_MODE_ULTRALOWPOWER:   return 5000;    break;
      case BMP085_MODE_STANDARD:        return 8000;    break;
      case BMP085_MODE_HIGHRES:         return 14000;   break;
      case BMP085_MODE_ULTRAHIGHRES:    return 26000;   break;
      default:                          return 26000;   break;
    }
}

float BMP085::thr_get_pressure(int32_t up)
{
    // Calculations from AdaFruit code
    // TODO: cleanup this mess

    int32_t compp = 0;
    int32_t x1, x2, b6, x3, b3, p;
    uint32_t b4, b7;

    // Pressure compensation
    b6 = m_b5 - 4000;
    x1 = (m_coeffs.b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (m_coeffs.ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((int32_t) m_coeffs.ac1) * 4 + x3) << m_mode) + 2) >> 2;
    x1 = (m_coeffs.ac3 * b6) >> 13;
    x2 = (m_coeffs.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (m_coeffs.ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = ((uint32_t) (up - b3) * (50000 >> m_mode));

    if (b7 < 0x80000000)
        p = (b7 << 1) / b4;
    else
        p = (b7 / b4) << 1;

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    compp = p + ((x1 + x2 + 3791) >> 4);

    // Assign compensated pressure value
    return compp / 100.0F;
}

int BMP085::get_reading(Bmp085ReadingType type, BMP085::Reading &value)
{
    if (type >= BMP085_TYPE_MAX || type < 0 || !m_initialized)
        return EINVAL;
    if (m_error)
        return m_error;
    int ret = m_readings[type].get_value(value);
    return ret;
}

int BMP085::thr_read_coeff()
{
    int res = 0;
    res = m_driver->i2c_read16(res, BMP085_ADDRESS, BMP085_REGISTER_CAL_AC1, &m_coeffs.ac1);
    res = m_driver->i2c_read16(res, BMP085_ADDRESS, BMP085_REGISTER_CAL_AC2, &m_coeffs.ac2);
    res = m_driver->i2c_read16(res, BMP085_ADDRESS, BMP085_REGISTER_CAL_AC3, &m_coeffs.ac3);
    res = m_driver->i2c_read16(res, BMP085_ADDRESS, BMP085_REGISTER_CAL_AC4, &m_coeffs.ac4);
    res = m_driver->i2c_read16(res, BMP085_ADDRESS, BMP085_REGISTER_CAL_AC5, &m_coeffs.ac5);
    res = m_driver->i2c_read16(res, BMP085_ADDRESS, BMP085_REGISTER_CAL_AC6, &m_coeffs.ac6);
    res = m_driver->i2c_read16(res, BMP085_ADDRESS, BMP085_REGISTER_CAL_B1, &m_coeffs.b1);
    res = m_driver->i2c_read16(res, BMP085_ADDRESS, BMP085_REGISTER_CAL_B2, &m_coeffs.b2);
    res = m_driver->i2c_read16(res, BMP085_ADDRESS, BMP085_REGISTER_CAL_MB, &m_coeffs.mb);
    res = m_driver->i2c_read16(res, BMP085_ADDRESS, BMP085_REGISTER_CAL_MC, &m_coeffs.mc);
    res = m_driver->i2c_read16(res, BMP085_ADDRESS, BMP085_REGISTER_CAL_MD, &m_coeffs.md);
    return res;
}

int32_t BMP085::thr_compute_b5(int32_t ut) 
{
    // Calculations from AdaFruit code
    const int32_t X1 = (ut - (int32_t)m_coeffs.ac6) * ((int32_t)m_coeffs.ac5) >> 15;
    const int32_t X2 = ((int32_t)m_coeffs.mc << 11) / (X1 + (int32_t)m_coeffs.md);
    return X1 + X2;
}



} // namespace robo
