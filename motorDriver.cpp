/*
 * Copyright (C) 2016 Tolga Ceylan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 * This file incorporates work covered by the following copyright and  
 * permission notice:
 */
/******************************************************************
 This is the library for the Adafruit Motor Shield V2 for Arduino. 
 It supports DC motors & Stepper motors with microstepping as well
 as stacking-support. It is *not* compatible with the V1 library!
 It will only work with https://www.adafruit.com/products/1483
 
 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source hardware
 by purchasing products from Adafruit!
 
 Written by Limor Fried/Ladyada for Adafruit Industries.
 BSD license, check license.txt for more information.
 All text above must be included in any redistribution.
 ******************************************************************/
#include "motorDriver.h"
#include "logger.h"
#include "i2c.h"
#include <errno.h>
#include <string.h>

namespace robo
{

#define LOW 0
#define HIGH 1

MotorDriver::MotorDriver(I2C *driver, Thread *thr)
    :
    m_freq(0),
    m_state(STATE_IDLE),
    m_pwm(driver, thr)
{
}

MotorDriver::~MotorDriver()
{
    shutdown();
}

void MotorDriver::shutdown()
{
    logger(LOG_INFO, "MotorDriver shutting down");
    m_pwm.shutdown();
    m_freq = 0;
    m_state = STATE_IDLE;
}

int MotorDriver::initialize(int address, uint16_t freq)
{
    logger(LOG_INFO, "MotorDriver initializing freq=%u", freq);
    int res = m_pwm.initialize(address, freq);
    if (res)
    {
        logger(LOG_ERROR, "MotorDriver pwm error addr=%d err=%d", address, res);
        return res;
    }

    m_freq = freq;
    m_state = STATE_INITIALIZE;
    return 0;
}

int MotorDriver::update(uint64_t now)
{
    int res = 0;

    switch (m_state)
    {
        case STATE_INITIALIZE:
        {
            if (!m_pwm.is_initialized())
                break;

            for (uint8_t i = 0; i < 16; ++i)
            {
                res = m_pwm.set_pwm(i, 0, 0);
                if (res)
                    break;
            }

            if (res)
                break;

            res = m_pwm.execute();
            if (res)
                break;

            m_state = STATE_INITIALIZING;
        }
        break;

        case STATE_INITIALIZING:
        {
            static uint64_t tmp = 0;
            if (tmp == 0)
                tmp = now;

            // Add some delay until we are fully initialized to avoid
            // multi threading perf issues (scanner and motor commands running
            // at the same time)
            if (Timer::get_elapsed_usec(tmp, now) < 5000)
                break;

            tmp = 0;
            m_state = STATE_INITIALIZED;
        }
        break;

        default:
        break;
    }

    if (!res)
        res = m_pwm.update(now);
    return res;
}

int MotorDriver::run_dc(DCMotor motor, uint8_t cmd)
{
    int res = 0;
    switch (cmd) 
    {
    case FORWARD:
        res = set_pin(motor.in2, LOW);  // take low first to avoid 'break'
        if (!res)
            res = set_pin(motor.in1, HIGH);
    break;
    case BACKWARD:
        res = set_pin(motor.in1, LOW);  // take low first to avoid 'break'
        if (!res)
            res = set_pin(motor.in2, HIGH);
    break;
    case RELEASE:
        res = set_pin(motor.in1, LOW);
        if (!res)
            res = set_pin(motor.in2, LOW);
    break;
    default:
        return EINVAL;
    }
    return res;
}

int MotorDriver::execute()
{
    return m_pwm.execute();
}

int MotorDriver::set_dc_speed(DCMotor motor, uint8_t speed) 
{
  return set_pwm(motor.pwm, speed * 16);
}

int MotorDriver::set_pwm(uint8_t pin, uint16_t value)
{
  if (value > 4095)
    return m_pwm.set_pwm(pin, 4096, 0);
  return m_pwm.set_pwm(pin, 0, value);
}

int MotorDriver::set_pin(uint8_t pin, uint8_t value)
{
  if (value == LOW)
    return m_pwm.set_pwm(pin, 0, 0);
  return m_pwm.set_pwm(pin, 4096, 0);
}

DCMotor MotorDriver::get_dc_motor(int num) 
{
    DCMotor motor;
    switch (num)
    {
        case 1:
            motor.pwm = 8; 
            motor.in2 = 9; 
            motor.in1 = 10; 
            motor.id = num;
        break;
        case 2:
            motor.pwm = 13; 
            motor.in2 = 12; 
            motor.in1 = 11; 
            motor.id = num;
        break;
        case 3:
            motor.pwm = 2; 
            motor.in2 = 3; 
            motor.in1 = 4;
            motor.id = num;
        case 4:
            motor.pwm = 7; 
            motor.in2 = 6; 
            motor.in1 = 5;
            motor.id = num;
        default:
        break;
    }

    return motor;
}

} // namespace robo
