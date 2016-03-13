/*
 * Copyright (C) 2016 Tolga Ceylan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 * This file incorporates work covered by the following copyright and  
 * permission notice:
 */
/***************************************************************************
  This is a library for the LSM303 Accelerometer and magnentometer/compass
  Designed specifically to work with the Adafruit LSM303DLHC Breakout
  These displays use I2C to communicate, 2 pins are required to interface.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#ifndef LSM303Mag_H__
#define LSM303Mag_H__

#include "mutexval.h"
#include "ahrs_sensor.h"

#include <stdint.h>
//
// Below code is based on Adafruit Provided Samples/Libs
//

namespace robo {

// I2C ADDRESS/BITS
#define LSM303_ADDRESS_MAG            (0x3C >> 1)         // 0011110x

// CHIP ID
#define LSM303_ID                     (0b11010100)

enum Lsm303MagRegisters
{
    LSM303_REGISTER_MAG_CRA_REG_M             = 0x00,
    LSM303_REGISTER_MAG_CRB_REG_M             = 0x01,
    LSM303_REGISTER_MAG_MR_REG_M              = 0x02,
    LSM303_REGISTER_MAG_OUT_X_H_M             = 0x03,
    LSM303_REGISTER_MAG_OUT_X_L_M             = 0x04,
    LSM303_REGISTER_MAG_OUT_Z_H_M             = 0x05,
    LSM303_REGISTER_MAG_OUT_Z_L_M             = 0x06,
    LSM303_REGISTER_MAG_OUT_Y_H_M             = 0x07,
    LSM303_REGISTER_MAG_OUT_Y_L_M             = 0x08,
    LSM303_REGISTER_MAG_SR_REG_Mg             = 0x09,
    LSM303_REGISTER_MAG_IRA_REG_M             = 0x0A,
    LSM303_REGISTER_MAG_IRB_REG_M             = 0x0B,
    LSM303_REGISTER_MAG_IRC_REG_M             = 0x0C,
    LSM303_REGISTER_MAG_TEMP_OUT_H_M          = 0x31,
    LSM303_REGISTER_MAG_TEMP_OUT_L_M          = 0x32
};

// MAGNETOMETER GAIN SETTINGS
enum Lsm303MagGain
{
    LSM303_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
    LSM303_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
    LSM303_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
    LSM303_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
    LSM303_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
    LSM303_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
    LSM303_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
};  

class I2C;
class Thread;

class LSM303Mag
{
  public:
    LSM303Mag(I2C *driver, Thread *thr);
    ~LSM303Mag();

    int initialize(uint64_t period, Lsm303MagGain gain, bool enableAutoRange);
    void shutdown();
    
    int update(uint64_t now);

    bool is_initialized() const
    {
        return m_initialized;
    }

    int get_reading(ahrs::Sensor &reading);

  private:

    static void s_initialize(void *arg, uint8_t *arg_blob);
    static void s_measure(void *arg, uint8_t *arg_blob);

    void thr_initialize(uint8_t arg);
    void thr_measure();
    void thr_read(ahrs::Sensor &reading);

    int thr_set_mag_gain(uint8_t gain);

    int check_exec_cmd(uint64_t now);

  private:

    I2C                 *m_driver;
    Thread              *m_thread;

    uint64_t            m_period;
    bool                m_initialized;
    bool                m_autoRange;

    // modified by I/O thread
    int                 m_error;
    Lsm303MagGain       m_gain;
    float               m_gauss_lsb_xy;  
    float               m_gauss_lsb_z;

    MutexVal<ahrs::Sensor>  m_reading;
};

} // namespace robo

#endif // LSM303Mag_H__
