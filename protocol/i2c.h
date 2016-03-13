/*
 * i2c.h
 *
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include <stdlib.h>

namespace robo {


class I2C
{
public:
	I2C();
	~I2C();

	int initialize(int bus);
	void shutdown();

	// generic functionality
	int i2c_set_slave_address(int address);
	int i2c_write(const uint8_t *data, int length);
	int i2c_read(uint8_t *data, int length, int &read_length);

	// widely used helper functions
    int i2c_read16(int addr, uint8_t reg, uint16_t *value);
    int i2c_read16(int addr, uint8_t reg, int16_t *value);
    int i2c_write16(int addr, uint8_t reg, uint8_t value);
	int i2c_write8(int addr, uint8_t reg);
    int i2c_read8(int addr, uint8_t reg, uint8_t *value);

    // Some inline helper functions to facilitate code clarity, if error
    // is set, then no-op, else perform the operation
	int i2c_write(int err, const uint8_t *data, int length)
	{
		return err ? err : i2c_write(data, length);
	}
	int i2c_read(int err, uint8_t *data, int length, int &read_length)
	{
		return err ? err : i2c_read(data, length, read_length);
	}

    int i2c_read16(int err, int addr, uint8_t reg, uint16_t *value)
    {
    	return err ? err : i2c_read16(addr, reg, value);
    }

    int i2c_read16(int err, int addr, uint8_t reg, int16_t *value)
    {
		return err ? err : i2c_read16(addr, reg, value);
    }

    int i2c_write16(int err, int addr, uint8_t reg, uint8_t value)
    {
		return err ? err : i2c_write16(addr, reg, value);
    }

    int i2c_write8(int err, int addr, uint8_t reg)
    {
		return err ? err : i2c_write8(addr, reg);
    }

    int i2c_read8(int err, int addr, uint8_t reg, uint8_t *value)
    {
		return err ? err : i2c_read8(addr, reg, value);
    }

private:
	int m_fd;
};

} // namespace robo
#endif /* I2C_H_ */
