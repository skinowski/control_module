#ifndef LSM303Accel_H__
#define LSM303Accel_H__

#include "mutexval.h"
#include "ahrs_sensor.h"

#include <stdint.h>

namespace robo {

// I2C ADDRESS/BITS
#define LSM303_ADDRESS_ACCEL          (0x32 >> 1)         // 0011001x

// CHIP ID
#define LSM303_ID                     (0b11010100)

// REGISTERS
enum Lsm303AccelRegisters
{                                                     // DEFAULT    TYPE
    LSM303_REGISTER_ACCEL_CTRL_REG1_A         = 0x20,   // 00000111   rw
    LSM303_REGISTER_ACCEL_CTRL_REG2_A         = 0x21,   // 00000000   rw
    LSM303_REGISTER_ACCEL_CTRL_REG3_A         = 0x22,   // 00000000   rw
    LSM303_REGISTER_ACCEL_CTRL_REG4_A         = 0x23,   // 00000000   rw
    LSM303_REGISTER_ACCEL_CTRL_REG5_A         = 0x24,   // 00000000   rw
    LSM303_REGISTER_ACCEL_CTRL_REG6_A         = 0x25,   // 00000000   rw
    LSM303_REGISTER_ACCEL_REFERENCE_A         = 0x26,   // 00000000   r
    LSM303_REGISTER_ACCEL_STATUS_REG_A        = 0x27,   // 00000000   r
    LSM303_REGISTER_ACCEL_OUT_X_L_A           = 0x28,
    LSM303_REGISTER_ACCEL_OUT_X_H_A           = 0x29,
    LSM303_REGISTER_ACCEL_OUT_Y_L_A           = 0x2A,
    LSM303_REGISTER_ACCEL_OUT_Y_H_A           = 0x2B,
    LSM303_REGISTER_ACCEL_OUT_Z_L_A           = 0x2C,
    LSM303_REGISTER_ACCEL_OUT_Z_H_A           = 0x2D,
    LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A     = 0x2E,
    LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A      = 0x2F,
    LSM303_REGISTER_ACCEL_INT1_CFG_A          = 0x30,
    LSM303_REGISTER_ACCEL_INT1_SOURCE_A       = 0x31,
    LSM303_REGISTER_ACCEL_INT1_THS_A          = 0x32,
    LSM303_REGISTER_ACCEL_INT1_DURATION_A     = 0x33,
    LSM303_REGISTER_ACCEL_INT2_CFG_A          = 0x34,
    LSM303_REGISTER_ACCEL_INT2_SOURCE_A       = 0x35,
    LSM303_REGISTER_ACCEL_INT2_THS_A          = 0x36,
    LSM303_REGISTER_ACCEL_INT2_DURATION_A     = 0x37,
    LSM303_REGISTER_ACCEL_CLICK_CFG_A         = 0x38,
    LSM303_REGISTER_ACCEL_CLICK_SRC_A         = 0x39,
    LSM303_REGISTER_ACCEL_CLICK_THS_A         = 0x3A,
    LSM303_REGISTER_ACCEL_TIME_LIMIT_A        = 0x3B,
    LSM303_REGISTER_ACCEL_TIME_LATENCY_A      = 0x3C,
    LSM303_REGISTER_ACCEL_TIME_WINDOW_A       = 0x3D
};
    
enum Lsm303AccelFrequency
{
    LSM303_ACCEL_100HZ                      = 0x57,
    LSM303_ACCEL_200HZ                      = 0x67,
    LSM303_ACCEL_400HZ                      = 0x77,
};

class I2C;
class Thread;

class LSM303Accel
{
  public:

    LSM303Accel(I2C *driver, Thread *thr);
    ~LSM303Accel();

    int initialize(uint64_t period, Lsm303AccelFrequency frequency);
    void shutdown();
    
    int update(uint64_t now);

    bool is_initialized() const
    {
        return m_initialized;
    }

    int get_reading(ahrs::Sensor &data);

  private:

    static void s_initialize(void *arg, uint8_t *arg_blob);
    static void s_measure(void *arg, uint8_t *arg_blob);

    void thr_initialize();
    void thr_measure();

    int check_exec_cmd(uint64_t now);

  private:

    I2C                 *m_driver;
    Thread              *m_thread;

    uint64_t            m_period;
    bool                m_initialized;
    int                 m_error;
    Lsm303AccelFrequency m_frequency;
    MutexVal<ahrs::Sensor>  m_reading;
};

} // namespace robo

#endif // LSM303Accel_H__
