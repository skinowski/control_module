/*
 * Copyright (C) 2016 Tolga Ceylan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 */
#include "thread.h"
#include "logger.h"
#include "utils.h"
#include "timer.h"

namespace robo {

void *Thread::entry(void *obj)
{
    Thread *thr = static_cast<Thread*>(obj);
    thr->process();
    return NULL;
}

Thread::Thread()
    :
    m_thread(),
    m_is_running(false),
    m_stop(false)
{
    clean_queue();
}
Thread::~Thread()
{
    shutdown();
}

int Thread::initialize(int priority, int policy)
{
    logger(LOG_INFO, "Thread initializing");

    if (m_is_running)
        return EINVAL;

    m_stop = false;
    int ret = 0;

    bool attr_init = false;
    bool mutex_init = false;
    bool cond_init = false;

    if (pthread_attr_init(&m_attr))
    {
        ret = errno;
        goto error;
    }
    attr_init = true;

    if (pthread_mutex_init(&m_mutex, NULL))
    {
        ret = errno;
        goto error;
    }
    mutex_init = true;

    if (pthread_cond_init(&m_cond, NULL))
    {
        ret = errno;
        goto error;
    }
    cond_init = true;

    struct sched_param param;
    
    memset(&param, 0, sizeof(param));
    param.sched_priority = clamp(
        priority, 
        sched_get_priority_min(policy),
        sched_get_priority_max(policy)
    );

    if (pthread_attr_setinheritsched(&m_attr, PTHREAD_EXPLICIT_SCHED) ||
        pthread_attr_setschedpolicy(&m_attr, policy) ||
        pthread_attr_setschedparam(&m_attr, &param) ||
        pthread_create(&m_thread, &m_attr, &Thread::entry, this)
    )
    {
        ret = errno;
        goto error;
    }

    logger(LOG_INFO, "Thread initialized tid=%zu pri=%d", m_thread, param.sched_priority);
    m_is_running = true;
    return 0;

    error:
    if (ret)
        logger(LOG_INFO, "Thread initialize error err=%d err=%d", ret, errno);

    if (mutex_init)
        pthread_mutex_destroy(&m_mutex);
    if (cond_init)
        pthread_cond_destroy(&m_cond);
    if (attr_init)
        pthread_attr_destroy(&m_attr);

    return ret;
}

void Thread::process()
{
    logger(LOG_INFO, "Thread running tid=%zu", m_thread);

    int res = 0;
    bool isLocked = false;

    int items = 0;
    while (!m_stop && !res)
    {
        res = pthread_mutex_lock(&m_mutex);
        if (res)
            break;
        isLocked = true;

        while (!res && !m_stop && m_queue.empty())
            res = pthread_cond_wait(&m_cond, &m_mutex);

        if (res || m_queue.empty() || m_stop)
            break;

        // copy & suck in all commands in queue
        items = 0;
        while (!m_queue.empty())
        {
            res = m_queue.pop(m_io_cmd[items++]);
            if (res)
                break;
        }

        res = pthread_mutex_unlock(&m_mutex);
        if (res)
            break;
        isLocked = false;

        for (int idx = 0; idx < items; ++idx)
            m_io_cmd[idx].fun(m_io_cmd[idx].arg, m_io_cmd[idx].arg_blob);
    }

    if (isLocked)
        pthread_mutex_unlock(&m_mutex);

    logger(res ? LOG_ERROR : LOG_INFO, "Thread exiting tid=%zu res=%d", m_thread, res);
}

void Thread::clean_queue()
{
    int res = pthread_mutex_lock(&m_mutex);
    if (res)
        return;
    m_queue.clear();
    pthread_mutex_unlock(&m_mutex);
}

int Thread::queue_cmd(const Thread::Cmd &cmd)
{
    if (!m_is_running)
        return EFAULT;

    int res = pthread_mutex_lock(&m_mutex);
    if (res)
        return res;

    res = m_queue.push(cmd);

    int res2 = pthread_mutex_unlock(&m_mutex);
    return res ? res : res2;
}

int Thread::signal_io()
{
    return pthread_cond_signal(&m_cond);
}

int Thread::check_signal_io()
{
    if (!m_is_running)
        return EFAULT;

    int res = pthread_mutex_lock(&m_mutex);
    if (res)
        return res;

    const bool do_signal = !m_queue.empty();

    res = pthread_mutex_unlock(&m_mutex);
    if (res)
        return res;

    return do_signal ? signal_io() : 0;
}

void Thread::shutdown()
{
    logger(LOG_INFO, "Thread shutting down");

    if (m_is_running)
    {
        m_stop = true;
        m_is_running = false;

        clean_queue();

        pthread_cond_signal(&m_cond);
        pthread_join(m_thread, NULL);

        pthread_mutex_destroy(&m_mutex);
        pthread_cond_destroy(&m_cond);
        pthread_attr_destroy(&m_attr);
    }

    m_queue.clear();
    m_stop = false;
    logger(LOG_INFO, "Thread shutdown");
}

} // namespace robo
 