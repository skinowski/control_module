/*
 * Copyright (C) 2016 Tolga Ceylan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 * This file incorporates work covered by the following copyright and  
 * permission notice:
 */
    //=====================================================================================================
    // MadgwickAHRS.c
    //=====================================================================================================
    //
    // Implementation of Madgwick's IMU and AHRS algorithms.
    // See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
    //
    // Date                 Author          Notes
    // 29/09/2011   SOH Madgwick    Initial release
    // 02/10/2011   SOH Madgwick    Optimised for reduced CPU load
    // 19/02/2012   SOH Madgwick    Magnetometer measurement is normalised

#include "ahrs_mahony.h"
#include <math.h>

namespace robo { namespace ahrs {

//---------------------------------------------------------------------------------------------------
inline float invSqrt(float x) 
{
    // Fast inverse square-root
    // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
//---------------------------------------------------------------------------------------------------
inline void normalize(Quaternion &quat)
{
    const float recipNorm = invSqrt(quat.q0 * quat.q0 + quat.q1 * quat.q1 + quat.q2 * quat.q2 + quat.q3 * quat.q3);
    
    quat.q0 *= recipNorm;
    quat.q1 *= recipNorm;
    quat.q2 *= recipNorm;
    quat.q3 *= recipNorm;
}
//---------------------------------------------------------------------------------------------------
inline void normalize(Sensor &input)
{
    const float recipNorm = invSqrt(input.x * input.x + input.y * input.y + input.z * input.z);

    input.x *= recipNorm;
    input.y *= recipNorm;
    input.z *= recipNorm;   
}
//---------------------------------------------------------------------------------------------------
void apply_error(float halfex, float halfey, float halfez, State &s, Sensor &g)
{
    // Compute and apply integral feedback if enabled
    if (s.twoKi > 0.0f) 
    {
        s.integralFBx += s.twoKi * halfex * (1.0f / s.sample_frequency);    // integral error scaled by Ki
        s.integralFBy += s.twoKi * halfey * (1.0f / s.sample_frequency);
        s.integralFBz += s.twoKi * halfez * (1.0f / s.sample_frequency);

        g.x += s.integralFBx;    // apply integral feedback
        g.y += s.integralFBy;
        g.z += s.integralFBz;
    }
    else 
    {
        s.integralFBx = 0.0f;    // prevent integral windup
        s.integralFBy = 0.0f;
        s.integralFBz = 0.0f;
    }

    // Apply proportional feedback
    g.x += s.twoKp * halfex;
    g.y += s.twoKp * halfey;
    g.z += s.twoKp * halfez;
}
//---------------------------------------------------------------------------------------------------
void integrate_gyro(State &s, Sensor g)
{
    // Integrate rate of change of quaternion
    g.x *= (0.5f * (1.0f / s.sample_frequency));        // pre-multiply common factors
    g.y *= (0.5f * (1.0f / s.sample_frequency));
    g.z *= (0.5f * (1.0f / s.sample_frequency));
    
    const float qa = s.orient.q0;
    const float qb = s.orient.q1;
    const float qc = s.orient.q2;
    
    s.orient.q0 += (-qb * g.x - qc * g.y - s.orient.q3 * g.z);
    s.orient.q1 += (qa * g.x + qc * g.z - s.orient.q3 * g.y);
    s.orient.q2 += (qa * g.y - qb * g.z + s.orient.q3 * g.x);
    s.orient.q3 += (qa * g.z + qb * g.y - qc * g.x); 
    
    normalize(s.orient);
}
//---------------------------------------------------------------------------------------------------
bool mahony_update(State &s, Sensor g, Sensor a, Sensor m)
{
    if (!g.is_good())
        return false;
    
    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if (!m.is_good() && a.is_good())
    {
        normalize(a);

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        const float halfvx = s.orient.q1 * s.orient.q3 - s.orient.q0 * s.orient.q2;
        const float halfvy = s.orient.q0 * s.orient.q1 + s.orient.q2 * s.orient.q3;
        const float halfvz = s.orient.q0 * s.orient.q0 - 0.5f + s.orient.q3 * s.orient.q3;
    
        // Error is sum of cross product between estimated and measured direction of gravity
        const float halfex = (a.y * halfvz - a.z * halfvy);
        const float halfey = (a.z * halfvx - a.x * halfvz);
        const float halfez = (a.x * halfvy - a.y * halfvx);

        apply_error(halfex, halfey, halfez, s, g);
    }
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    else if (a.is_good())
    {
        normalize(a);
        normalize(m);

        // Auxiliary variables to avoid repeated arithmetic
        const float q0q0 = s.orient.q0 * s.orient.q0;
        const float q0q1 = s.orient.q0 * s.orient.q1;
        const float q0q2 = s.orient.q0 * s.orient.q2;
        const float q0q3 = s.orient.q0 * s.orient.q3;
        const float q1q1 = s.orient.q1 * s.orient.q1;
        const float q1q2 = s.orient.q1 * s.orient.q2;
        const float q1q3 = s.orient.q1 * s.orient.q3;
        const float q2q2 = s.orient.q2 * s.orient.q2;
        const float q2q3 = s.orient.q2 * s.orient.q3;
        const float q3q3 = s.orient.q3 * s.orient.q3;

        // Reference direction of Earth's magnetic field
        const float hx = 2.0f * (m.x * (0.5f - q2q2 - q3q3) + m.y * (q1q2 - q0q3) + m.z * (q1q3 + q0q2));
        const float hy = 2.0f * (m.x * (q1q2 + q0q3) + m.y * (0.5f - q1q1 - q3q3) + m.z * (q2q3 - q0q1));

        const float bx = sqrt(hx * hx + hy * hy);
        const float bz = 2.0f * (m.x * (q1q3 - q0q2) + m.y * (q2q3 + q0q1) + m.z * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        const float halfvx = q1q3 - q0q2;
        const float halfvy = q0q1 + q2q3;
        const float halfvz = q0q0 - 0.5f + q3q3;
        const float halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        const float halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        const float halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
    
        // Error is sum of cross product between estimated direction and measured direction of field vectors
        const float halfex = (a.y * halfvz - a.z * halfvy) + (m.y * halfwz - m.z * halfwy);
        const float halfey = (a.z * halfvx - a.x * halfvz) + (m.z * halfwx - m.x * halfwz);
        const float halfez = (a.x * halfvy - a.y * halfvx) + (m.x * halfwy - m.y * halfwx);

        apply_error(halfex, halfey, halfez, s, g);
    }
    
    integrate_gyro(s, g);
    return true;
}

}} // namespace robo::mahony_ahrs