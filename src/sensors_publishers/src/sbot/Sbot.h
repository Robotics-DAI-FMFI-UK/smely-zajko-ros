#ifndef PROJECT_SBOT_H
#define PROJECT_SBOT_H

#include "AbstractSbot.h"

#include"ros/ros.h"

class Sbot : public AbstractSbot {
    int fdR[2];
    int fdW[2];
    int child;
    pthread_mutex_t m_read;
    message_types::SbotMsg result;
public:

    void init();

    void readData();

    message_types::SbotMsg getData();
};


#endif //PROJECT_SBOT_H
