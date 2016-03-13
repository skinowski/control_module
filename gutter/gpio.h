/*
 * Copyright (C) 2016 Tolga Ceylan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 */
#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>
#include <stdlib.h>

namespace robo {

/*
#define PUD_OFF 0
#define PUD_DOWN 1
#define PUD_UP 2
#define DIRECTION_INPUT 1
#define DIRECTION_OUTPUT 2


class GPIO
{
public:
    GPIO();
    ~GPIO();

    bool is_initialized() const
    {
        return m_fd != -1;
    }

    int initialize(const char *channel, int direction, int pud, int initial);
    void shutdown();

    int fetch_value(int &value);
    int update_value(int value);

private:

    int gpio_unexport(int gpio);
    int gpio_export(int gpio);
    int gpio_open(int gpio, int &fd);
    int gpio_set_direction(int gpio, int direction);

private:
    int m_gpio;
    int m_fd;
};
*/

} // namespace robo
#endif /* GPIO_H_ */
