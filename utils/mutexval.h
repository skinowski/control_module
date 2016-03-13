/*
 * MutexVal.h
 *
 */

#ifndef MUTEX_VAL_H__
#define MUTEX_VAL_H__

#include <errno.h>
#include <pthread.h>
#include <stdint.h>

namespace robo {

template <typename T>
class MutexVal
{
public:

	MutexVal()
		:
		m_value(),
		m_mutex(),
		m_initialized(false),
		m_error(0)
	{
	    if (pthread_mutex_init(&m_mutex, NULL))
	    	m_error = errno;
	    else
	    	m_initialized = true;
	}

	~MutexVal()
	{
		if (m_initialized)
			pthread_mutex_destroy(&m_mutex);
	}

	int set_value(const T &value)
	{
		if (m_error)
			return m_error;

		if (pthread_mutex_lock(&m_mutex))
		{
			m_error = errno;
			return m_error;
		}

		m_value = value;
		
		if (pthread_mutex_unlock(&m_mutex))
			m_error = errno;
		return m_error;
	}

	int get_value(T &value)
	{
		if (m_error)
			return m_error;

		if (pthread_mutex_lock(&m_mutex))
		{
			m_error = errno;
			return m_error;
		}

		value = m_value;
		
		if (pthread_mutex_unlock(&m_mutex))
			m_error = errno;
		return m_error;
	}

private:
	T 				m_value;
    pthread_mutex_t m_mutex;
    bool            m_initialized;
    int 			m_error;

};

} // namespace robo
#endif /* MUTEX_VAL_H__ */
