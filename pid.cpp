
#include "pid.h"
#include "timer.h"
#include "logger.h"

namespace robo {
    
void pid_init(Pid &pid, int id, double kp, double ki, double kd)
{
    pid.Kp = kp;
    pid.Ki = ki;
    pid.Kd = kd;
    pid.error = 0.0f;
    pid.time = 0;
    pid.integral = 0.0f;

    pid.target = 0;
    pid.output = 0.0f;
    pid.actual = 0;

    pid.id = id;
}

void pid_update(Pid &pid, uint64_t now)
{
    const double error = pid.target - pid.actual;

    if (pid.time == 0)
    {
        pid.time = now;
        pid.error = error;
    }

    // WARNING: Code here is probably broken

    // not enough time (none) has elapsed.
    if (pid.time >= now)
        return;

    // msecs
    const double dt = Timer::get_elapsed_usec(pid.time, now) / 1000.0l;

    pid.integral += error * dt;
    
    const double derivative = (error - pid.error) / dt;

    pid.output = 
        pid.Kp * error + 
        pid.Ki * pid.integral + 
        pid.Kd * derivative;

    pid.error    = error;
    pid.time     = now;

    return;
}

void pid_logger(const Pid &pid)
{
    logger(LOG_INFO, "PID id=%d intg=%f err=%f target=%d actual=%d output=%f", 
        pid.id,
        pid.integral,
        pid.error,
        pid.target,
        pid.actual,
        pid.output
    );
}

} // namespace robo