
#include "gpio.h"
#include "logger.h"

namespace robo {
/*
GPIO::GPIO()
	:
	m_gpio(-1)	
	m_fd(-1)
{
	
}
GPIO::~GPIO()
{
	shutdown();
}

void GPIO::shutdown()
{
	logger(LOG_INFO, "gpio %d shutting down", m_gpio);
	if (!is_initialized())
		return;

	if (m_gpio != -1)
	{
		gpio_unexport(m_gpio);
		m_gpio = -1;
	}

	if (m_fd != -1)
	{
		::close(m_fd);
		m_fd = -1;
	}
}


int GPIO::initialize(int gpio, int direction, int pud, int initial)
{
	logger(LOG_INFO, "gpio initializing %d direction=%d pud=%d initial=%d", gpio, direction, pud, initial);
	if (is_initialized())
		return EFAULT;

	if (direction != DIRECTION_OUTPUT && direction != DIRECTION_INPUT)
		return EINVAL;

	if (direction == DIRECTION_OUTPUT)
		pud = PUD_OFF;

	if (pud != PUD_OFF && pud != PUD_DOWN && pud != PUD_UP)
		return EINVAL;

	int ret = 0;

	ret = gpio_export(gpio);
	if (ret)
		goto error;

	ret = gpio_set_direction(gpio, direction);
	if (ret)
		goto error;

	ret = gpio_open(gpio, m_fd);
	if (ret)
		goto error;

	if (direction == DIRECTION_OUTPUT)
		ret = update_value(initial, true);
	else
		ret = update_value(pud, true);
	if (ret)
		goto error;

	m_gpio = gpio;

	return ret;
	error:
		shutdown();
		return ret;
}

int GPIO::fetch_value(int &value, bool isBlock)
{
	char ch;
	try_again:
	const int len = HANDLE_EINTR(::pread(m_fd, &ch, 1, 0));
	if (len == 1)
	{
		value = ch != '0' ? 1 : 0;
		return 0;
	}
	if (isBlock)
	{
		if ((len == -1 && errno == EAGAIN)) || len == 0)
			goto try_again;
	}
	return (len == 0) ? EAGAIN : errno;
}

int GPIO::update_value(int value, bool isBlock)
{
	const char value = value ? '1' : '0';
	try_again:
	const int len = HANDLE_EINTR(::pwrite(fd, &value, 1, 0));
	if (len == 1)
		return 0;
	if (isBlock)
	{
		if ((len == -1 && errno == EAGAIN)) || len == 0)
			goto try_again;
	}
	return (len == 0) ? EAGAIN : errno;
}

int GPIO::gpio_open(int gpio, int &fd)
{
	char buf[128];
    snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", gpio);
    fd = ::open(buf, O_RDWR | O_NONBLOCK);
    if (fd < 0)
    	return errno;
    return 0;
}

int GPIO::gpio_set_direction(int gpio, int direction)
{
	char buf[128];
	snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/direction", gpio);
	if (direction == DIRECTION_OUTPUT)
		return write_buf_to_file(buf, "out", 3);
	if (direction == DIRECTION_INPUT)
		return write_buf_to_file(buf, "in", 2);
	return EINVAL;
}

int GPIO::gpio_unexport(int gpio)
{
	return write_int_to_file("/sys/class/gpio/unexport", gpio);
}

int GPIO::gpio_export(int gpio)
{
	return write_int_to_file("/sys/class/gpio/export", gpio);
}
*/
} // namespace robo