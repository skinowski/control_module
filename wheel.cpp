/*
 * wheel.cpp
 *
 *  Created on: Dec 18, 2014
 *      Author: tceylan
 */
#include "wheel.h"
#include "logger.h"
#include <errno.h>
#include <stdlib.h>
#include <assert.h>

namespace robo {

Wheel::Wheel(MotorDriver *driver)
	:
	m_driver(driver),
	m_motor()
{
	assert(driver);
}

Wheel::~Wheel()
{
	shutdown();
}

int Wheel::initialize(int id)
{
	logger(LOG_INFO, "Wheel initializing id=%u", id);

	m_motor = MotorDriver::get_dc_motor(id);
	
	const int res = m_motor.is_valid() ? 0 : EFAULT;
	if (res)
		logger(LOG_ERROR, "Wheel initialize err=%d", res);
	return res;
}

uint8_t Wheel::get_dc_speed(int speed)
{
	return abs(speed);
}

uint8_t Wheel::get_cmd(int speed)
{
	if (speed > 0)
		return FORWARD;
	else if (speed < 0)
		return BACKWARD;
	return RELEASE;
}

int Wheel::set_speed(int speed)
{
	const uint8_t cmd 		= get_cmd(speed);
	const uint8_t dc_speed 	= get_dc_speed(speed);

	logger(LOG_INFO, "Wheel set_speed id=%u speed=%u cmd=%u", m_motor.id, dc_speed, cmd);

	int res = 0;
	if (dc_speed)
		res = m_driver->set_dc_speed(m_motor, dc_speed);

	if (!res)
		res = m_driver->run_dc(m_motor, cmd);
	return res;
}

void Wheel::shutdown()
{
	logger(LOG_INFO, "Wheel shutdown id=%u", m_motor.id);
	set_speed(0);
	m_motor = DCMotor();
}

} // namespace robo





