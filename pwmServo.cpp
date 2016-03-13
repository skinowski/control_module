
#include "pwmServo.h"
#include "logger.h"
#include "i2c.h"
#include "thread.h"
#include "utils.h"

#include <errno.h>
#include <math.h>
#include <assert.h>

namespace robo
{

#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD

// Static forwarders
void PWMServo::s_reset(void *arg, uint8_t *arg_blob)
{
	PWMServo *me = static_cast<PWMServo *>(arg);
	me->thr_reset();
}

void PWMServo::s_set_pwm_freq1(void *arg, uint8_t *arg_blob)
{
	PWMServo *me = static_cast<PWMServo *>(arg);
	me->thr_set_pwm_freq1();
}

void PWMServo::s_set_pwm_freq2(void *arg, uint8_t *arg_blob)
{
	PWMServo *me = static_cast<PWMServo *>(arg);
	me->thr_set_pwm_freq2();
}

void PWMServo::s_set_pwm(void *arg, uint8_t *arg_blob)
{
	PWMServo *me = static_cast<PWMServo *>(arg);
	me->thr_set_pwm(arg_blob);
}
// end of static forwarders

PWMServo::PWMServo(I2C *driver, Thread *thr)
	:
	m_address(0),
	m_freq(0.0f),
	m_driver(driver),
	m_thread(thr),
	m_initialized(false),
	m_io_ready(false),
	m_oldmode(0)
{
	assert(driver);
	assert(thr);
}

PWMServo::~PWMServo()
{
	shutdown();
}

void PWMServo::shutdown()
{
	logger(LOG_INFO, "PWMServo shutting down");
	if (!m_thread || !m_thread->is_running() || !m_initialized)
		return;

	Thread::Cmd cmd;
	cmd.fun = &PWMServo::s_reset;
	cmd.arg = this;

	int res = m_thread->queue_cmd(cmd);
	if (!res)
		res = m_thread->signal_io();
	if (res)
		logger(LOG_ERROR, "PWMServo shutdown queue_cmd error err=%d", res);

	m_initialized = false;
}

int PWMServo::initialize(int address, float freq)
{
	logger(LOG_INFO, "PWMServo initializing address=0x%x", address);
	if (!m_thread->is_running() || m_initialized)
		return EINVAL;

	m_address = address;
	m_freq = freq;
	m_initialized = true;

	Thread::Cmd cmd;
	cmd.fun = &PWMServo::s_set_pwm_freq1;
	cmd.arg = this;

	int res = 0;

	res = m_thread->queue_cmd(cmd);
	if (!res)
		res = m_thread->signal_io();
	if (res)
		logger(LOG_ERROR, "PWMServo initialize error address=0x%x freq=%f err=%d", 
			address, freq, res);
	return res;
}

uint8_t PWMServo::thr_get_prescale() const
{
	float prescale_val = 25000000;
	prescale_val /= 4096;
	prescale_val /= m_freq;
	prescale_val -= 1;
	const uint8_t prescale = floor(prescale_val + 0.5);
	return prescale;	
}

int PWMServo::execute()
{
	if (!m_thread || !m_thread->is_running())
		return EINVAL;
	return m_thread->signal_io();
}

int PWMServo::update(uint64_t now)
{
	if (m_start_pwm_freq2 && !m_io_ready)
	{
		static uint64_t prev = 0;
		if (prev == 0)
			prev = now;

		if (Timer::get_elapsed_usec(prev, now) > 5000)
		{
			Thread::Cmd cmd;
			cmd.fun = &PWMServo::s_set_pwm_freq2;
			cmd.arg = this;

			int res = m_thread->queue_cmd(cmd);
			if (!res)
				res = m_thread->signal_io();
			prev = now; // this will schedule this again, after 5ms *if* thread fails.
			return res;
		}
	}

	return 0;
}

void PWMServo::thr_reset()
{
	int res = 0;
	const uint8_t data[2] = { PCA9685_MODE1, 0x0 };

	m_io_ready = false;

	res = m_driver->i2c_set_slave_address(m_address);
	res = m_driver->i2c_write(res, data, 2);
	
	if (res)
		logger(LOG_ERROR, "PWMServo thr_reset error err=%d", res);
}

void PWMServo::thr_set_pwm_freq1()
{
	uint8_t data[2];
	int len = 0;
	uint8_t newmode = 0;
	int res = 0;

	m_io_ready = false;

	data[0] = PCA9685_MODE1;
	data[1] = 0x0;

	res = m_driver->i2c_set_slave_address(m_address);
	res = m_driver->i2c_write(res, data, 2);
	res = m_driver->i2c_read(res, data, 1, len);

	if (!res && len != 1)
		res = EINVAL;

	m_oldmode = data[0];
	newmode = (m_oldmode & 0x7F) | 0x10; // sleep

	data[0] = PCA9685_MODE1;
	data[1] = newmode;
	res = m_driver->i2c_write(res, data, 2);  // goto sleep

	data[0] = PCA9685_PRESCALE;
	data[1] = thr_get_prescale();
	res = m_driver->i2c_write(res, data, 2);  // set the prescaler

	data[0] = PCA9685_MODE1;
	data[1] = m_oldmode;
	res = m_driver->i2c_write(res, data, 2);  // goto sleep

	if (!res)
		m_start_pwm_freq2 = true;
	else
		logger(LOG_ERROR, "PWMServo thr_set_pwm_freq1 error err=%d", res);
}

void PWMServo::thr_set_pwm_freq2()
{
	uint8_t data[2];
	int res = 0;

	m_start_pwm_freq2 = false;

	// This sets the MODE1 register to turn on auto increment
	data[0] = PCA9685_MODE1;
	data[1] = m_oldmode | 0xa1;

	res = m_driver->i2c_set_slave_address(m_address);
	res = m_driver->i2c_write(res, data, 2);  

	if (!res)
		m_io_ready = true;
	else
		logger(LOG_ERROR, "PWMServo thr_set_pwm_freq2 error err=%d", res);
}

int PWMServo::set_pwm(uint8_t num, uint16_t on, uint16_t off) 
{
	if (!m_thread || !m_thread->is_running() || !m_initialized || !m_io_ready)
		return EINVAL;

	Thread::Cmd cmd;

	cmd.fun = &PWMServo::s_set_pwm;
	cmd.arg = this;

	cmd.arg_blob[0] = LED0_ON_L + 4 * num;
	cmd.arg_blob[1] = on;
	cmd.arg_blob[2] = on >> 8;
	cmd.arg_blob[3] = off;
	cmd.arg_blob[4] = off >> 8;

	const int res = m_thread->queue_cmd(cmd);
	if (res)
		logger(LOG_ERROR, "PWMServo set_pwm error err=%d", res);
	return res;
}

void PWMServo::thr_set_pwm(uint8_t *data)
{
	int res = 0;

	res = m_driver->i2c_set_slave_address(m_address);
	res = m_driver->i2c_write(res, data, 5);

	if (res)
		logger(LOG_ERROR, "PWMServo thr_set_pwm error err=%d", res);
}

} // namespace robo



