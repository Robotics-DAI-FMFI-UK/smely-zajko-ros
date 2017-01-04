#ifndef PROJECT_HOKUYO_H
#define PROJECT_HOKUYO_H

#include "AbstractHokuyo.h"

#include"ros/ros.h"
#include <arpa/inet.h>

class Hokuyo : public AbstractHokuyo {
    static const int BUFFER_SIZE = 5000;
    int sockfd;
    int *data;
    unsigned char read_buf[BUFFER_SIZE];
    pthread_mutex_t m_read;

public:
    static const int RANGE_DATA_COUNT = 1081;

    void init();

    void readData();

    Int32MultiArray getData();
};


#endif //PROJECT_HOKUYO_H
