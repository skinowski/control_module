/*
 * Copyright (C) 2016 Tolga Ceylan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 */
#ifndef CIRC_ARRAY_H_
#define CIRC_ARRAY_H_

#include <errno.h>

namespace robo {

template <typename T, size_t S>
class CircArray
{
public:

    CircArray()
        :
        m_queue_start(0),
        m_queue_items(0)
    {
    }

    int push(const T &t)
    {
        if (m_queue_items >= S)
            return ENOBUFS;

        const size_t idx = (m_queue_start + m_queue_items) % S;
        m_queue[idx] = t;
        ++m_queue_items;
        return 0;
    }

    int pop(T &t)
    {
        if (m_queue_items == 0)
            return ENOBUFS;

        t = m_queue[m_queue_start];

        m_queue_start = ++m_queue_start % S;
        --m_queue_items;
        return 0;
    }

    bool empty() const
    {
        return m_queue_items == 0;
    }

    void clear()
    {
        m_queue_items = 0;
        m_queue_start = 0;
    }

private:
    T         m_queue[S];
    size_t     m_queue_start;
    size_t     m_queue_items;
};

} // namespace robo
#endif /* CIRC_ARRAY_H_ */
