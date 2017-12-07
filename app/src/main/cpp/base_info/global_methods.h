//
// Created by shydh on 8/11/17.
//

#ifndef AGVP_GLOBAL_METHODS_H
#define AGVP_GLOBAL_METHODS_H

#include <sys/time.h>
#include <sys/select.h>
#include <time.h>
#include <stdio.h>

namespace base_info
{
    /*seconds: the seconds; mseconds: the micro seconds*/
    void setTimer(int seconds, int mseconds, void(* func)())
    {
        struct timeval temp;

        temp.tv_sec = seconds;
        temp.tv_usec = mseconds;

        select(0, NULL, NULL, NULL, &temp);
        func();

        return ;
    }
}

#endif //AGVP_GLOBAL_METHODS_H
