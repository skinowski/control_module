/*
 * Copyright (C) 2016 Tolga Ceylan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 */
#include "poller.h"
#include "logger.h"
#include "utils.h"

#include <string.h>
#include <unistd.h>
#include <errno.h>

namespace robo {
    

Poller::Poller()
{
    clear();
}

Poller::~Poller()
{
    shutdown();
}

void Poller::shutdown()
{
    logger(LOG_INFO, "Poller shutting down");
    clear();
}

int Poller::initialize()
{
    logger(LOG_INFO, "Poller initializing");
    clear();
    return 0;
}

void Poller::clear()
{
    memset(m_cb, 0, sizeof(m_cb));
    memset(m_fd, 0, sizeof(m_fd));
    memset(m_cb_data, 0, sizeof(m_cb_data));
}

int Poller::add_entity(int fd, int events, int &id, Poller::Callback cb, void *cb_data)
{
    if (fd < 0 || !events || !cb)
        return EINVAL;

    int loc = -1;
    for (int idx = 0; idx < MAX_POLL_ITEMS; ++idx)
    {
        if (!m_cb[idx])
        {
            loc = idx;
            break;
        }
    }

    if (loc == -1)
        return ENOBUFS;

    logger(LOG_INFO, "Poller adding id=%d fd=%d", fd, loc + 1);

    m_fd[loc].fd         = fd;
    m_fd[loc].events     = events;

    m_cb[loc]             = cb;
    m_cb_data[loc]        = cb_data;

    id = loc + 1;
    return 0;
}

int Poller::rm_entity(int id)
{
    --id;
    if (id >= MAX_POLL_ITEMS || id < 0 || !m_cb[id])
        return EINVAL;

    logger(LOG_INFO, "Poller removing id=%d fd=%d", id + 1, m_fd[id].fd);

    m_fd[id].fd         = -1;
    m_fd[id].events     = 0;
    m_cb[id]             = 0;

    return 0;
}

int Poller::update(uint64_t maxWait)
{
    int ret = HANDLE_EINTR(::poll(m_fd, MAX_POLL_ITEMS, maxWait / 1000));
    if (ret < 0)
        return errno;

    for (int idx = 0; ret > 0 && idx < MAX_POLL_ITEMS; ++idx)
    {
        if (m_fd[idx].revents)
        {
            --ret;
            if (m_cb[idx])
                m_cb[idx](m_cb_data[idx], idx + 1, m_fd[idx].fd, m_fd[idx].revents);
        }
    }
    return 0;
}

} // namespace robo