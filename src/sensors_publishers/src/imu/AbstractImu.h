#ifndef PROJECT_ABSTRACTIMU_H
#define PROJECT_ABSTRACTIMU_H

#include "sensor_msgs/Imu.h"

class AbstractImu {
public:
    virtual void init() = 0;

    virtual void readData() = 0;

    virtual sensor_msgs::Imu getData() = 0;
};


#endif //PROJECT_ABSTRACTIMU_H
