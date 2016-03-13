/*
 * thread.h
 *
 */

#ifndef THREAD_H_
#define THREAD_H_

#include <pthread.h>
#include "circarray.h"
#include <string.h>
#include <stdint.h>

namespace robo {

class Thread
{
public:

    enum
    {
        QUEUE_SIZE = 256,
        CMD_ARG_BLOB_SIZE = 64
    };

    typedef void (*CmdFun)(void *arg, uint8_t *arg_blob);

    struct Cmd
    {
        CmdFun fun;
        void *arg;
        uint8_t arg_blob[CMD_ARG_BLOB_SIZE];

        Cmd()
            :
            fun(0),
            arg(0)
        {
            memset(arg_blob, 0, sizeof(arg_blob));
        }
    };


    Thread();
    ~Thread();

    int initialize(int priority, int policy);
    void shutdown();

    int queue_cmd(const Cmd &cmd);
    int signal_io();
    int check_signal_io();

    bool is_running() const
    {
        return m_is_running;
    }

private:
    void process();
    static void *entry(void *obj);
    void clean_queue();

private:
    pthread_t                     m_thread;
    bool                         m_is_running;
    bool                         m_stop;

    pthread_mutex_t             m_mutex;
    pthread_cond_t                m_cond;
    pthread_attr_t                 m_attr;

    CircArray<Cmd, QUEUE_SIZE>     m_queue;
    Cmd                         m_io_cmd[QUEUE_SIZE];
};

} // namespace robo
#endif /* THREAD_H_ */
