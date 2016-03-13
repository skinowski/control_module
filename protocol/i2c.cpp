/*
 */

#include "i2c.h"
#include "logger.h"

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/errno.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <stdio.h>
#include <string.h>

#include "utils.h"

namespace robo
{

int i2c_smbus_access(int fh, uint8_t read_write, uint8_t command, int size, union i2c_smbus_data *data)
{
    i2c_smbus_ioctl_data args;

    args.read_write = read_write;
    args.command = command;
    args.size = size;
    args.data = data;

    return HANDLE_EINTR(::ioctl(fh, I2C_SMBUS, &args));
}

I2C::I2C()
    :
    m_fd(-1)
{
}

I2C::~I2C()
{
    shutdown();
}

void I2C::shutdown()
{
	logger(LOG_INFO, "i2c shutting down");
	if (m_fd != -1)
	{
		HANDLE_EINTR(::close(m_fd));
		m_fd = -1;
	}
}

int I2C::initialize(int bus)
{
    logger(LOG_INFO, "i2c initializing bus=%d", bus);
    if (m_fd != -1)
    {
        logger(LOG_ERROR, "i2c already initialized bus=%d", bus);
		    return EINVAL;
    }

    char filepath[32];
    snprintf(filepath, 32, "/dev/i2c-%u", bus);
    m_fd = HANDLE_EINTR(::open(filepath, O_RDWR | O_NONBLOCK));
   
    const int ret = m_fd == -1 ? errno : 0;

    if (ret)
        logger(LOG_ERROR, "i2c open error bus=%d err=%d", bus, ret);
    return ret;
}

int I2C::i2c_set_slave_address(int address)
{
    if (m_fd == -1)
		    return EBADF;

    const int res = HANDLE_EINTR(::ioctl(m_fd, I2C_SLAVE_FORCE, address));
    return res ? errno : 0;
}

int I2C::i2c_write(const uint8_t *data, int length)
{
  	if (m_fd == -1)
     		return EBADF;
  	if (length > I2C_SMBUS_BLOCK_MAX + 1)
  	   	return EFBIG;

    union i2c_smbus_data msg;
    
    const uint8_t command = data[0];

    --length;

    msg.block[0] = length;
    memcpy(msg.block + 1, data + 1, length);

    const int res = i2c_smbus_access(m_fd, I2C_SMBUS_WRITE, command, I2C_SMBUS_I2C_BLOCK_DATA, &msg);
    return res ? errno : 0;
}

int I2C::i2c_read(uint8_t *data, int length, int &read_length)
{
  	const int res = HANDLE_EINTR(::read(m_fd, data, length));
  	if (res < 0)
     		return errno;
  	read_length = res;
  	return 0;
}

int I2C::i2c_read8(int addr, uint8_t reg, uint8_t *value)
{
    int res = i2c_set_slave_address(addr);
    if (res)
       return res;

    res = i2c_write(&reg, sizeof(reg));
    if (res)
       return res;

    int len = 0;
    res = i2c_read(value, sizeof(*value), len);
    if (res)
        return res;
    if (len != sizeof(*value))
        return EFAULT;
    return 0;
}

int I2C::i2c_read16(int addr, uint8_t reg, uint16_t *value)
{
    uint8_t data[2] = { reg, 0 };
    int res = 0;
    res = i2c_set_slave_address(addr);
    if (res)
       return res;

    res = i2c_write(data, 1);
    if (res)
       return res;

    int len = 0;
    res = i2c_read(data, sizeof(data), len);
    if (res)
       return res;
    if (len != sizeof(data))
        return EFAULT;

    *value = (data[0] << 8) | data[1];
    return 0;
}

int I2C::i2c_read16(int addr, uint8_t reg, int16_t *value)
{
    uint16_t temp = 0;
    const int ret = i2c_read16(addr, reg, &temp);
    if (!ret)
       *value = static_cast<int16_t>(temp);
    return ret;
}

int I2C::i2c_write8(int addr, uint8_t reg)
{
    int res = i2c_set_slave_address(addr);
    if (res)
       return res;
    return i2c_write(&reg, sizeof(reg));
}

int I2C::i2c_write16(int addr, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = { reg, value };
    int res = i2c_set_slave_address(addr);
    if (res)
       return res;
    return i2c_write(data, sizeof(data));
}



} // namespace robo



