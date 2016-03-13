/*
 * scheduler.cpp
 *
 *  Created on: Dec 31, 2014
 *      Author: tceylan
 */

#include "scheduler.h"
#include "logger.h"
#include "utils.h"
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <string.h> // memset

#include <unistd.h>
#include <sys/syscall.h>

namespace robo {

int set_scheduler(int policy, int priority)
{
    struct sched_param param;
    
    memset(&param, 0, sizeof(param));

    param.sched_priority = clamp(
        priority, 
        sched_get_priority_min(policy),
        sched_get_priority_max(policy)
    );

    const pid_t me = syscall(SYS_gettid);

    const int res = sched_setscheduler(me, policy, &param);
    if (res != 0)
    {
        logger(LOG_ERROR, "Scheduler initialize failed for tid=%d policy=%d pri=%d errno=%d", 
            me, policy, param.sched_priority, errno);
        return errno;
    }
    logger(LOG_INFO, "Scheduler tid=%d policy=%d pri=%d", me, policy, param.sched_priority);
    return 0;
}

} // namespace robo


