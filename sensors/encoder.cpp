/*
 * encoder.cpp
 *
 *      Author: tceylan
 */

#include "encoder.h"

#include "logger.h"
#include "thread.h"
#include "i2c.h"
#include "timer.h"

#include <assert.h>

namespace robo {

#define ENCODER_ADDRESS (0x20)

void Encoder::s_measure(void *arg, uint8_t *arg_blob)
{
    Encoder *me = static_cast<Encoder *>(arg);
    me->thr_measure();
}

Encoder::Encoder(I2C *driver, Thread *thr)
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

Encoder::~Encoder()
{
    shutdown();
}

int Encoder::initialize(uint64_t period)
{
    logger(LOG_INFO, "Encoder initializing");
    if (m_initialized || !period)
        return EINVAL;

    m_initialized   = true;
    m_period = period;
    return 0;
}

void Encoder::shutdown()
{
    logger(LOG_INFO, "Encoder shutting down");
    if (!m_thread->is_running() || !m_initialized)
        return;
    m_initialized = false;
    m_error         = 0;
}

int Encoder::update(uint64_t now)
{
    if (!m_initialized)
        return 0;

    if (m_error)
        return m_error;

    return check_exec_cmd(now);
}

int Encoder::check_exec_cmd(uint64_t now)
{
    assert(m_period);
    static uint64_t prev = now;

    // check elapsed since
    if (Timer::get_elapsed_usec(prev, now) <= m_period)
        return 0;

    prev = now;

    Thread::Cmd cmd;
    cmd.arg = this;
    cmd.fun = &Encoder::s_measure;

    int res = m_thread->queue_cmd(cmd);
    if (res)
        logger(LOG_INFO, "Encoder check_exec_cmd error err=%d err=%d", res, errno);
    return res;
}

void Encoder::thr_measure()
{
    uint8_t data[16] = { 0 };
    Encoder::Reading reading;
    int len = 0;

    reading.error = m_driver->i2c_set_slave_address(ENCODER_ADDRESS);
    reading.error = m_driver->i2c_read(reading.error, data, sizeof(data), len);

    if (!reading.error && len != sizeof(data)) {
        reading.error = EIO;
        logger(LOG_ERROR, "Encoder thr_measure error len=%d", len);
    }

    memcpy(&reading.left, &data[0], 4);
    memcpy(&reading.right, &data[4], 4);
    memcpy(&reading.isr_count, &data[8], 4);
    memcpy(&reading.cmd_count, &data[12], 4);

    reading.time = Timer().get_now();
    int res = m_reading.set_value(reading);
    if (res) {
        logger(LOG_ERROR, "Encoder thr_measure error err=%d", res);
        m_error = res;
    }
}

int Encoder::get_reading(Encoder::Reading &reading)
{
    if (m_error)
        return m_error;
    int ret = m_reading.get_value(reading);
    return ret;
}



} // namespace robo
