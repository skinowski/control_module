/*
 * poller.h
 *
 *  Created on: Dec 26, 2014
 *      Author: tceylan
 */

#ifndef POLLER_H_
#define POLLER_H_

#include <poll.h>
#include <stdint.h>

namespace robo {

class Poller
{
public:
    Poller();
    ~Poller();

    // We don't need this dynamic for this project.
    enum {
        MAX_POLL_ITEMS = 4,
    };

    typedef void (*Callback)(void *cb_data, int id, int fd, int revents);

    int add_entity(int fd, int events, int &id, Callback cb, void *cb_data);
    int rm_entity(int id);

    int update(uint64_t maxWait);

    int initialize();
    void shutdown();

private:
    void clear();

private:
    Callback         m_cb[MAX_POLL_ITEMS];
    void *            m_cb_data[MAX_POLL_ITEMS];
    struct pollfd     m_fd[MAX_POLL_ITEMS];
};

} // namespace robo

#endif /* SCANNER_H_ */
