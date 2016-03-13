/*
 * motor_driver.h
 *
 */

#ifndef MotorDriver_H__
#define MotorDriver_H__

#include <stdint.h>
#include "pwmServo.h"

namespace robo {

#define FORWARD 2
#define BACKWARD 1
#define RELEASE 4
	
#define LEFT 1
#define RIGHT 2

struct DCMotor
{
	uint8_t pwm;
	uint8_t in1; 
	uint8_t in2;
	uint8_t id;

	DCMotor()
		:
		pwm(0),
		in1(0),
		in2(0),
		id(0)
	{}

	bool is_valid() const
	{
		return id != 0;
	}
};


class I2C;
class Thread;

class MotorDriver
{
public:
	enum
	{
		STATE_IDLE = 0,
		STATE_INITIALIZE = 1,
		STATE_INITIALIZING = 2,
		STATE_INITIALIZED = 3,
	};

	MotorDriver(I2C *driver, Thread *thr);
	~MotorDriver();

	static DCMotor get_dc_motor(int index);

	int initialize(int address, uint16_t freq);
	void shutdown();

	int update(uint64_t now);

	int run_dc(DCMotor motor, uint8_t cmd);
	int set_dc_speed(DCMotor motor, uint8_t speed);
	int execute();

	bool is_initialized() const
	{
		return m_state == STATE_INITIALIZED;
	}

private:
	int set_pwm(uint8_t pin, uint16_t value);
	int set_pin(uint8_t pin, uint8_t value);

private:
	uint16_t 	m_freq;
	int 		m_state;
	PWMServo 	m_pwm;

};

} // namespace robo
#endif /* MotorDriver_H__ */
