/*
 * pwm_servo.h
 *
 */

#ifndef PWMServo_H__
#define PWMServo_H__

#include <stdint.h>
#include <stdlib.h>
#include "timer.h"

namespace robo {

class I2C;
class Thread;

class PWMServo
{
public:

	PWMServo(I2C *driver, Thread *thr);
	~PWMServo();

	int initialize(int address, float freq);
	void shutdown();
	int update(uint64_t now);

	int set_pwm(uint8_t num, uint16_t on, uint16_t off);
	int execute();

	bool is_initialized() const
	{
		return m_initialized && m_io_ready;
	}

private:
	uint8_t thr_get_prescale() const;

	void thr_reset();
	void thr_set_pwm_freq1();
	void thr_set_pwm_freq2();
	void thr_set_pwm(uint8_t *data);

	static void s_reset(void *arg, uint8_t *arg_blob);
	static void s_set_pwm_freq1(void *arg, uint8_t *arg_blob);
	static void s_set_pwm_freq2(void *arg, uint8_t *arg_blob);
	static void s_set_pwm(void *arg, uint8_t *arg_blob);

private:
	int m_address;
	float m_freq;

	I2C *m_driver;
	Thread *m_thread;

	bool 	m_initialized;
	bool 	m_io_ready;
	bool 	m_start_pwm_freq2;
	uint8_t m_oldmode;
};
} // namespace robo

#endif /* PWMServo_H__ */