/*
 * Copyright (C) 2016 Tolga Ceylan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 */
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include "robo.h"

int running = 1;

void sig_handler(int signo)
{
    if (signo == SIGINT)
        running = 0;
}

int main (int argc, char **argv)
{
    int demo_num = 0;
    if (argc == 2)
        demo_num = atoi(argv[1]);

    robo::Robot *robot = new robo::Robot(demo_num);

    int ret = robot ? robot->initialize() : ENOMEM;
    if (!ret)
    {
        signal(SIGINT, sig_handler);

        while ((running == 1) && (ret == 0))
            ret = robot->update();

        robot->shutdown();
    }

    // give 40 msecs to components to shutdown
    usleep(40000);
    delete robot;
    robot = 0;

done:
    return ret;
}
