/*
 * Copyright (C) 2016 Tolga Ceylan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 */
#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include <sched.h>

namespace robo {

    int set_scheduler(int policy, int priority);
}



#endif /* SCHEDULER_H_ */
