#ifndef PROJECT_SBOT_H
#define PROJECT_SBOT_H

#include"ros/ros.h"
#include "message_types/SbotMsg.h"

class Sbot {
    int fdW[2];
    int child;
    pthread_mutex_t m_read;
    message_types::SbotMsg result;
    int running;

    friend void *base_module_thread(void *arg);

public:
    int fdR[2];

    int init();

    void getData(message_types::SbotMsg *result);

    void shutdown();

};


#endif //PROJECT_SBOT_H
