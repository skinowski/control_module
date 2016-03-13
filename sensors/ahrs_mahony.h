#ifndef AHRS_MAHONY_H__
#define AHRS_MAHONY_H__

#include "ahrs_sensor.h"

namespace robo { namespace ahrs {
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
//
struct Quaternion
{
	float q0;			
	float q1;
	float q2;
	float q3;

	Quaternion()
		:
		q0(1.0f),
		q1(0.0f),
		q2(0.0f),
		q3(0.0f)
	{}
};

struct State
{
	Quaternion 	orient;			// sensor frame relative to auxiliary frame
	float 		twoKp;			// 2 * proportional gain (Kp)
	float 		twoKi;			// 2 * integral gain (Ki)
	float 		integralFBx; 	// integral error terms scaled by Ki
	float 		integralFBy;
	float 		integralFBz; 
	float 		sample_frequency;		// sample frequency in Hz

	State()
		:
		twoKp(2.0f * 0.5f),
		twoKi(2.0f * 0.0f),
		orient(),
		sample_frequency(512.0f)
	{}
};

//
// Update the provided state with gyro, accelerator and magnetometer
// data.
bool mahony_update(State &s, Sensor g, Sensor a, Sensor m);

}} // namespace robo::mahony_ahrs

#endif // AHRS_MAHONY_H__
