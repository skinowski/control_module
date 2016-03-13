/*
 * Copyright (C) 2016 Tolga Ceylan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 * This file incorporates work covered by the following copyright and  
 * permission notice:
 */
    //=====================================================================================================
    // MadgwickAHRS.h
    //=====================================================================================================
    //
    // Implementation of Madgwick's IMU and AHRS algorithms.
    // See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
    //
    // Date                 Author          Notes
    // 29/09/2011   SOH Madgwick    Initial release
    // 02/10/2011   SOH Madgwick    Optimised for reduced CPU load
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
    Quaternion     orient;            // sensor frame relative to auxiliary frame
    float         twoKp;            // 2 * proportional gain (Kp)
    float         twoKi;            // 2 * integral gain (Ki)
    float         integralFBx;     // integral error terms scaled by Ki
    float         integralFBy;
    float         integralFBz; 
    float         sample_frequency;        // sample frequency in Hz

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
