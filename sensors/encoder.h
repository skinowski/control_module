/*
 * encoder.h
 *
 *      Author: tceylan
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "mutexval.h"

#include <stdint.h>

namespace robo {

class I2C;
class Thread;


class Encoder
{
public:
    struct Reading
    {
        uint32_t left;
        uint32_t right;
        uint32_t isr_count;
        uint32_t cmd_count;
        int      error;
        uint64_t time;

        Reading()
            :
            left(0),
            right(0),
            isr_count(0),
            cmd_count(0),
            error(0),
            time(0)
        {}

        void clear_data()
        {
            left = 0;
            right = 0;
            isr_count = 0;
            cmd_count = 0;
            error = 0;
        }

        bool is_error() const
        {
            return error;
        }
    };

    Encoder(I2C *driver, Thread *thr);
    ~Encoder();

    int initialize(uint64_t period);
    void shutdown();

    int update(uint64_t now);

    bool is_initialized() const
    {
        return m_initialized;
    }

    int get_reading(Encoder::Reading &sample);

private:
    // no copy
    Encoder(const Encoder &);
    Encoder& operator=(const Encoder &);

    static void s_measure(void *arg, uint8_t *arg_blob);

    void thr_measure();

    int check_exec_cmd(uint64_t now);

private:
    I2C                     *m_driver;
    Thread                  *m_thread;
    uint64_t                m_period;
    bool                     m_initialized;
    int                     m_error;
    MutexVal<Encoder::Reading> m_reading;
};


} // namespace robo

#endif /* ENCODER_H_ */
