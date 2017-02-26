#ifndef PROJECT_HOKUYO_H
#define PROJECT_HOKUYO_H

#include "AbstractHokuyo.h"

#include"ros/ros.h"
#include <arpa/inet.h>

#define address "169.254.0.10"
#define port 10490

class Hokuyo : public AbstractHokuyo {
    static const int BUFFER_SIZE = 5000;
    int sockfd;
    int *data;
    unsigned char read_buf[BUFFER_SIZE];
    pthread_mutex_t m_read;

public:

    void init();

    void readData();

    Int32MultiArray getData();
};


#endif //PROJECT_HOKUYO_H
