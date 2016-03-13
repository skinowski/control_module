/*
 * pid.h
 *
 *      Author: tceylan
 */

#ifndef PID_H_
#define PID_H_

#include <stdint.h>

namespace robo {

struct Pid
{
	int id;

	double Kp;
	double Ki;
	double Kd;

	double error;		// last error
	uint64_t time;		// last time
	double integral;	

	int target;		// target state
	int actual;		// current state

	double output;		// output (energy)
};

void pid_init(Pid &pid, int id, double kp, double ki, double kd);
void pid_update(Pid &pid, uint64_t now);
void pid_logger(const Pid &pid);


} // namespace robo

#endif /* PID_H_ */
