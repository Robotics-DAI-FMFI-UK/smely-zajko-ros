#ifndef PROJECT_ABSTRACTGPS_H
#define PROJECT_ABSTRACTGPS_H

#include "sensor_msgs/NavSatFix.h"

class AbstractGps {

public:
    virtual void init() = 0;

    virtual void readData() = 0;

    virtual sensor_msgs::NavSatFix getData() = 0;

};


#endif //PROJECT_ABSTRACTGPS_H
