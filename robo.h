/*
 * robo.h
 *
 *  Created on: Dec 18, 2014
 *      Author: tceylan
 */

#ifndef ROBO_H_
#define ROBO_H_

#include "i2c.h"
#include "car.h"
#include "thread.h"
#include "poller.h"

// sensors
#include "bmp085.h"
#include "l3gd20.h"
#include "lsm303accel.h"
#include "lsm303mag.h"
#include "encoder.h"

#include "sensor_state.h"

namespace robo {

//
// Robot state for sonar search gap and advance
// algorithm
//
struct State
{
    bool isTriggerAdvance;

    int distance;

    uint64_t    maxRun;
    uint64_t    minFrameTime;
    uint64_t    warmupDeadline;
    bool        isDone;

    State();
    void reset(uint64_t start_deadline = 0);
    void start();
    void print() const;
};

//
// Stats for robot such as frame rate, etc.
//
struct Stats
{
    uint64_t m_maxFrameTime;
    uint64_t m_minFrameTime;
    uint64_t m_avgFrameTime;
    uint64_t m_numOfFrames;
    uint64_t m_avgFrameSmoother;
    uint64_t m_displayPeriod;
    uint64_t m_totalFrameTime;

    Stats();
    void reset();
    void print() const;
    bool update_stats(uint64_t elapsed);
};

class Robot
{
public:
    Robot(int demo_num);
    ~Robot();
    int initialize();
    void shutdown();
    int update();

private:
    // no copy
    Robot(const Robot &);
    Robot& operator=(const Robot &);

    int update_car(uint64_t now);
    int update_driver(uint64_t now);
    int update_sensor_state(uint64_t now);
    
    int update_bmp085(uint64_t now);
    int update_l3gd20(uint64_t now);
    int update_lsm303accel(uint64_t now);
    int update_lsm303mag(uint64_t now);
    int update_encoder(uint64_t now);
    int update_io_thread(uint64_t now);

    int update_poller(uint64_t start, uint64_t &elapsed);

    void error_shut_sequence();

private:
    const int m_demo_num;

    robo::Timer         m_timer;

    robo::Poller         m_poller;

    robo::Thread         m_thread1;

    robo::I2C             m_i2c1;

    robo::MotorDriver     m_driver;

    robo::Car             m_car;

    robo::BMP085         m_bmp085;
    robo::L3GD20         m_l3gd20;
    robo::LSM303Accel     m_lsm303accel;
    robo::LSM303Mag     m_lsm303mag;
    robo::Encoder         m_encoder;

    robo::Stats         m_stats;
    robo::SensorState    m_sensor_state;

    State                 m_state;

};

} // namespace robo

#endif /* ROBO_H_ */
