/*
 * car.h
 *
 *  Created on: Dec 26, 2014
 *      Author: tceylan
 */

#ifndef CAR_H_
#define CAR_H_

#include "wheel.h"
#include "pid.h"
#include <stdint.h>

namespace robo {

class SensorState;

class Car
{
public:
    Car(robo::MotorDriver *driver, SensorState *sensors);
    ~Car();

    void shutdown();
    int initialize(int left_id, int right_id, uint64_t period);

    int update(uint64_t now);

    int turn(int angle);
    int run(int distance);

    void stop();

private:
    void reset_state();
    int update_wheel(robo::Wheel &wheel, int &speed, robo::Pid &pid, bool &isExecute);
    int update_pid(uint64_t now, Pid &pid_left, Pid &pid_right);

    static int convert_to_speed(const Pid &pid);

private:
    robo::MotorDriver *m_driver;

    robo::SensorState *m_sensors;

    robo::Wheel m_left;
    robo::Wheel m_right;

    int m_leftSpeed;
    int m_rightSpeed;

    Pid m_leftPid;
    Pid m_rightPid;

    uint64_t m_previous;
    uint64_t m_period;
};
} // namespace robo




#endif /* CAR_H_ */
