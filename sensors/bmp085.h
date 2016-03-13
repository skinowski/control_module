/*
 * Copyright (C) 2016 Tolga Ceylan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 * This file incorporates work covered by the following copyright and  
 * permission notice:
 */
/***************************************************************************
  This is a library for the BMP085 pressure sensor
  Designed specifically to work with the Adafruit BMP085 or BMP180 Breakout 
  ----> http://www.adafruit.com/products/391
  ----> http://www.adafruit.com/products/1603
  These displays use I2C to communicate, 2 pins are required to interface.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#ifndef BMP085_H__
#define BMP085_H__

#include <mutexval.h>

namespace robo {

// REGISTERS
enum
{
    BMP085_REGISTER_CAL_AC1            = 0xAA,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_AC2            = 0xAC,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_AC3            = 0xAE,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_AC4            = 0xB0,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_AC5            = 0xB2,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_AC6            = 0xB4,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_B1             = 0xB6,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_B2             = 0xB8,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_MB             = 0xBA,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_MC             = 0xBC,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_MD             = 0xBE,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CHIPID             = 0xD0,
    BMP085_REGISTER_VERSION            = 0xD1,
    BMP085_REGISTER_SOFTRESET          = 0xE0,
    BMP085_REGISTER_CONTROL            = 0xF4,
    BMP085_REGISTER_TEMPDATA           = 0xF6,
    BMP085_REGISTER_PRESSUREDATA       = 0xF6,
    BMP085_REGISTER_READTEMPCMD        = 0x2E,
    BMP085_REGISTER_READPRESSURECMD    = 0x34
};

// MODE SETTINGS
enum
{
    BMP085_MODE_ULTRALOWPOWER          = 0,
    BMP085_MODE_STANDARD               = 1,
    BMP085_MODE_HIGHRES                = 2,
    BMP085_MODE_ULTRAHIGHRES           = 3
};

//CALIBRATION DATA
struct Bmp085_calib_data
{
    int16_t  ac1;
    int16_t  ac2;
    int16_t  ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t  b1;
    int16_t  b2;
    int16_t  mb;
    int16_t  mc;
    int16_t  md;
};

class I2C;
class Thread;

enum Bmp085ReadingType
{
    BMP085_TYPE_PRESSURE       = 0,
    BMP085_TYPE_TEMPERATURE    = 1,
    BMP085_TYPE_MAX            = 2,
};

/* Pressure range: 300..1100 hPa with 0.01 hPa resolution */

class BMP085
{
  public:
    struct Reading
    {
        float       value;
        uint64_t    time;
        Reading();
    };

    BMP085(I2C *driver, Thread *thr);
    ~BMP085();

    int initialize(uint64_t period);
    void shutdown();
    
    int update(uint64_t now);

    bool is_initialized() const
    {
        return m_initialized;
    }

    int get_reading(Bmp085ReadingType type, BMP085::Reading &data);

  private:

    static void s_initialize(void *arg, uint8_t *arg_blob);
    static void s_measure(void *arg, uint8_t *arg_blob);

    void thr_initialize();
    void thr_measure(uint8_t arg);

    int32_t thr_compute_b5(int32_t ut);
    int thr_read_coeff();
    int thr_read_temperature(float &temp, int32_t &b5);
    int thr_read_pressure(float &pressure);
    int thr_request_temperature();
    int thr_request_pressure();
    float thr_get_pressure(int32_t up);
    int thr_set_metric(Bmp085ReadingType type, float value);

    uint64_t get_pressure_wait(uint8_t mode) const;
    int check_exec_cmd(uint64_t now);

  private:

    static const size_t MAX_STATES = 3;

    I2C                 *m_driver;
    Thread              *m_thread;

    const uint8_t       m_mode;
    bool                m_initialized;

    size_t              m_state;
    uint64_t            m_state_wait[MAX_STATES];

    MutexVal<BMP085::Reading>   m_readings[BMP085_TYPE_MAX];

    // warning: modified only by I/O thread
    Bmp085_calib_data   m_coeffs;
    int32_t             m_b5;
    int                 m_error;
};

} // namespace robo

#endif // BMP085_H__
