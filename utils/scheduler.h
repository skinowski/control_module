/*
 * scheduler.h
 *
 *  Created on: Dec 31, 2014
 *      Author: tceylan
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include <sched.h>

namespace robo {

    int set_scheduler(int policy, int priority);
}



#endif /* SCHEDULER_H_ */
