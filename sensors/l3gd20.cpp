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
#include "l3gd20.h"

#include "logger.h"
#include "thread.h"
#include "i2c.h"
#include "timer.h"

#include <assert.h>

#define SENSORS_DPS_TO_RADS               (0.017453293F)          /**< Degrees/s to rad/s multiplier */

#define GYRO_SENSITIVITY_250DPS  (0.00875F)    // Roughly 22/256 for fixed point match
#define GYRO_SENSITIVITY_500DPS  (0.0175F)     // Roughly 45/256
#define GYRO_SENSITIVITY_2000DPS (0.070F)      // Roughly 18/256

namespace robo {

void L3GD20::s_initialize(void *arg, uint8_t *arg_blob)
{
    L3GD20 *me = static_cast<L3GD20 *>(arg);
    me->thr_initialize();
}

void L3GD20::s_measure(void *arg, uint8_t *arg_blob)
{
    L3GD20 *me = static_cast<L3GD20 *>(arg);
    me->thr_measure();
}

L3GD20::L3GD20(I2C *driver, Thread *thr)
    :
    m_driver(driver),
    m_thread(thr),
    m_period(0),
    m_range(GYRO_RANGE_250DPS),
    m_initialized(false),
    m_autoRange(false),
    m_error(0)
{
    assert(driver);
    assert(thr);
}

L3GD20::~L3GD20()
{
    shutdown();
}
 
int L3GD20::initialize(uint64_t period, GyroRange range, bool enableAutoRange)
{
    logger(LOG_INFO, "L3GD20 initializing");
    if (!m_thread->is_running() || m_initialized || !period)
        return EINVAL;

    m_initialized   = true;
    m_period        = period;
    m_range         = range;
    m_autoRange     = enableAutoRange;

    Thread::Cmd cmd;
    cmd.arg = this;
    cmd.fun = &L3GD20::s_initialize;

    int res = m_thread->queue_cmd(cmd);
    if (res)
        logger(LOG_INFO, "L3GD20 initialize error err=%d err=%d", res, errno);
    return res;
}

void L3GD20::shutdown()
{
    logger(LOG_INFO, "L3GD20 shutting down");
    if (!m_thread->is_running() || !m_initialized)
        return;

    // nothing to do really...
    m_initialized   = false;
    m_error         = 0;
}

int L3GD20::update(uint64_t now)
{
    if (!m_initialized)
        return 0;

    if (m_error)
        return m_error;

    return check_exec_cmd(now);
}

int L3GD20::check_exec_cmd(uint64_t now)
{
    assert(m_period);
    static uint64_t prev = now;

    // check elapsed since
    if (Timer::get_elapsed_usec(prev, now) <= m_period)
        return 0;

    prev = now;

    Thread::Cmd cmd;
    cmd.arg = this;
    cmd.fun = &L3GD20::s_measure;

    int res = m_thread->queue_cmd(cmd);
    if (res)
        logger(LOG_INFO, "L3GD20 check_exec_cmd error err=%d err=%d", res, errno);
    return res;
}

int L3GD20::thr_adjust_reset(GyroRange range)
{
    int res = 0;

    /* Set CTRL_REG1 (0x20)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
    7-6  DR1/0     Output data rate                                   00
    5-4  BW1/0     Bandwidth selection                                00
     3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
     2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
     1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
     0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

    /* Reset then switch to normal mode and enable all three channels */
    uint8_t data[2] = { L3GD20_ADDRESS, 0x00 };

    res = m_driver->i2c_write16(res, L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0x00);
    res = m_driver->i2c_write16(res, L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0x0F);

    /* ------------------------------------------------------------------ */

    /* Set CTRL_REG2 (0x21)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
    5-4  HPM1/0    High-pass filter mode selection                    00
    3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */

    /* Nothing to do ... keep default values */
    /* ------------------------------------------------------------------ */

    /* Set CTRL_REG3 (0x22)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
     7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
     6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
     5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
     4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
     3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
     2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
     1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
     0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

    /* Nothing to do ... keep default values */
    /* ------------------------------------------------------------------ */

    /* Set CTRL_REG4 (0x23)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
     7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
     6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
    5-4  FS1/0     Full scale selection                               00
                                  00 = 250 dps
                                  01 = 500 dps
                                  10 = 2000 dps
                                  11 = 2000 dps
     0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */

    /* Adjust resolution if requested */
    switch(range)
    {
        case GYRO_RANGE_250DPS:
            res = m_driver->i2c_write16(res, L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG4, 0x00);
        break;
        case GYRO_RANGE_500DPS:
            res = m_driver->i2c_write16(res, L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG4, 0x10);
        break;
        case GYRO_RANGE_2000DPS:
            res = m_driver->i2c_write16(res, L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG4, 0x20);
        break;
    }

    /* ------------------------------------------------------------------ */

    /* Set CTRL_REG5 (0x24)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
     7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
     6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
     4  HPen      High-pass filter enable (0=disable,1=enable)        0
    3-2  INT1_SEL  INT1 Selection config                              00
    1-0  OUT_SEL   Out selection config                               00 */

    /* Nothing to do ... keep default values */
    /* ------------------------------------------------------------------ */
    res = m_driver->i2c_write16(res, L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG5, 0x80);

    return res;    
}

void L3GD20::thr_initialize()
{
    int res = 0;

    /* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
    uint8_t id = 0;
    res = m_driver->i2c_read8(res, L3GD20_ADDRESS, GYRO_REGISTER_WHO_AM_I, &id);

    if (!res && (id != L3GD20_ID) && (id != L3GD20H_ID))
        res = EFAULT;

    if (!res)
        res = thr_adjust_reset(m_range);

    if (res)
    {
        m_error = res;
        logger(LOG_ERROR, "L3GD20 thr_initialize error err=%d", res);
    }
    else
    {
        logger(LOG_INFO, "L3GD20 thr_initialize OK");
    }
}

void L3GD20::thr_read(ahrs::Sensor &reading)
{
    int len = 0;
    uint8_t data[6] = { GYRO_REGISTER_OUT_X_L | 0x80 };

    reading.error = m_driver->i2c_write8(reading.error, L3GD20_ADDRESS, data[0]);
    reading.error = m_driver->i2c_read(reading.error, data, sizeof(data), len);

    if (!reading.error && len != sizeof(data))
        reading.error = EIO;

    if (reading.error)
        return;

    uint8_t xlo = data[0];
    uint8_t xhi = data[1];
    uint8_t ylo = data[2];
    uint8_t yhi = data[3];
    uint8_t zlo = data[4];
    uint8_t zhi = data[5];

    /* Shift values to create properly formed integer (low byte first) */
    reading.x = (int16_t)(xlo | (xhi << 8));
    reading.y = (int16_t)(ylo | (yhi << 8));
    reading.z = (int16_t)(zlo | (zhi << 8));
}

void L3GD20::thr_measure()
{
    ahrs::Sensor reading;
  
    // Copy.. We are not allowed to change this. (See initialize() on main thread)
    GyroRange range         = m_range;
    const bool isAutoRange  = m_autoRange;

    while (!reading.error)
    {
        thr_read(reading);
        if (reading.error)
            break;
   
        /* Make sure the sensor isn't saturating if auto-ranging is enabled */
        if (!isAutoRange)
            break;

        /* Check if the sensor is saturating or not */
        /* All values are withing range? */
        if (reading.x < 32760 && reading.x > -32760 &&
             reading.y < 32760 && reading.y > -32760 &&
             reading.z < 32760 && reading.z > -32760)
            break;

        /* Saturating .... increase the range if we can */
        if (range == GYRO_RANGE_500DPS)
        {
            /* Push the range up to 2000dps */
            range = GYRO_RANGE_2000DPS;
            reading.error = thr_adjust_reset(range);
        }
        else if (range == GYRO_RANGE_250DPS)
        {
            /* Push the range up to 500dps */
            range = GYRO_RANGE_500DPS;
            reading.error = thr_adjust_reset(range);
        }
        else
        {
            break; // done
        }
    }
  
    thr_compensate_reading(reading, range);
    thr_convert_to_radian(reading);
      
    reading.time = Timer().get_now();
    int res = m_reading.set_value(reading);
    if (res)
        m_error = res;
}

int L3GD20::get_reading(ahrs::Sensor &reading)
{
    if (m_error)
        return m_error;
    int ret = m_reading.get_value(reading);
    return ret;
}

void L3GD20::thr_convert_to_radian(ahrs::Sensor &reading)
{
    if (reading.error)
        return;

    /* Convert values to rad/s */
    reading.x *= SENSORS_DPS_TO_RADS;
    reading.y *= SENSORS_DPS_TO_RADS;
    reading.z *= SENSORS_DPS_TO_RADS;
}

void L3GD20::thr_compensate_reading(ahrs::Sensor &reading, GyroRange range)
{
    if (reading.error)
        return;

    float comp = 0.0f;

    /* Compensate values depending on the resolution */
    switch(range)
    {
        case GYRO_RANGE_250DPS:
            comp = GYRO_SENSITIVITY_250DPS;
        break;
        case GYRO_RANGE_500DPS:
            comp = GYRO_SENSITIVITY_500DPS;
        break;
        case GYRO_RANGE_2000DPS:
            comp = GYRO_SENSITIVITY_2000DPS;
        break;
        default:
            return; // no-op
        break;
    }

    reading.x *= comp;
    reading.y *= comp;
    reading.z *= comp;
}

} // namespace robo
