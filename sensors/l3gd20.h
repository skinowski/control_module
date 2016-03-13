/*
 * Copyright (C) 2016 Tolga Ceylan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 * This file incorporates work covered by the following copyright and  
 * permission notice:
 */
/***************************************************
  This is a library for the L3GD20 GYROSCOPE
  Designed specifically to work with the Adafruit L3GD20 Breakout 
  ----> https://www.adafruit.com/products/1032
  These sensors use I2C or SPI to communicate, 2 pins (I2C) 
  or 4 pins (SPI) are required to interface.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
  Written by Kevin "KTOWN" Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#ifndef L3GD20_H__
#define L3GD20_H__

#include "mutexval.h"
#include "ahrs_sensor.h"

#include <stdint.h>

namespace robo {

// I2C ADDRESS/BITS AND SETTINGS
#define L3GD20_ADDRESS           (0x6B)        // 1101011
#define L3GD20_ID                0xD4
#define L3GD20H_ID               0xD7

// REGISTERS
enum GyroRegisters
{                                             // DEFAULT    TYPE
    GYRO_REGISTER_WHO_AM_I            = 0x0F,   // 11010100   r
    GYRO_REGISTER_CTRL_REG1           = 0x20,   // 00000111   rw
    GYRO_REGISTER_CTRL_REG2           = 0x21,   // 00000000   rw
    GYRO_REGISTER_CTRL_REG3           = 0x22,   // 00000000   rw
    GYRO_REGISTER_CTRL_REG4           = 0x23,   // 00000000   rw
    GYRO_REGISTER_CTRL_REG5           = 0x24,   // 00000000   rw
    GYRO_REGISTER_REFERENCE           = 0x25,   // 00000000   rw
    GYRO_REGISTER_OUT_TEMP            = 0x26,   //            r
    GYRO_REGISTER_STATUS_REG          = 0x27,   //            r
    GYRO_REGISTER_OUT_X_L             = 0x28,   //            r
    GYRO_REGISTER_OUT_X_H             = 0x29,   //            r
    GYRO_REGISTER_OUT_Y_L             = 0x2A,   //            r
    GYRO_REGISTER_OUT_Y_H             = 0x2B,   //            r
    GYRO_REGISTER_OUT_Z_L             = 0x2C,   //            r
    GYRO_REGISTER_OUT_Z_H             = 0x2D,   //            r
    GYRO_REGISTER_FIFO_CTRL_REG       = 0x2E,   // 00000000   rw
    GYRO_REGISTER_FIFO_SRC_REG        = 0x2F,   //            r
    GYRO_REGISTER_INT1_CFG            = 0x30,   // 00000000   rw
    GYRO_REGISTER_INT1_SRC            = 0x31,   //            r
    GYRO_REGISTER_TSH_XH              = 0x32,   // 00000000   rw
    GYRO_REGISTER_TSH_XL              = 0x33,   // 00000000   rw
    GYRO_REGISTER_TSH_YH              = 0x34,   // 00000000   rw
    GYRO_REGISTER_TSH_YL              = 0x35,   // 00000000   rw
    GYRO_REGISTER_TSH_ZH              = 0x36,   // 00000000   rw
    GYRO_REGISTER_TSH_ZL              = 0x37,   // 00000000   rw
    GYRO_REGISTER_INT1_DURATION       = 0x38    // 00000000   rw
};    

// OPTIONAL SPEED SETTINGS
enum GyroRange
{
    GYRO_RANGE_250DPS  = 250,
    GYRO_RANGE_500DPS  = 500,
    GYRO_RANGE_2000DPS = 2000
};

class I2C;
class Thread;

class L3GD20
{
  public:
    L3GD20(I2C *driver, Thread *thr);
    ~L3GD20();

    int initialize(uint64_t period, GyroRange range, bool enableAutoRange);
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

    void thr_initialize();
    void thr_measure();
    void thr_read(ahrs::Sensor &reading);

    int thr_adjust_reset(GyroRange range);
    void thr_convert_to_radian(ahrs::Sensor &reading);
    void thr_compensate_reading(ahrs::Sensor &reading, GyroRange range);

    int check_exec_cmd(uint64_t now);

  private:

    I2C                 *m_driver;
    Thread              *m_thread;

    uint64_t            m_period;
    GyroRange           m_range;
    bool                m_initialized;
    bool                m_autoRange;
    int                 m_error;

    MutexVal<ahrs::Sensor>  m_reading;
};

} // namespace robo

#endif // L3GD20_H__
