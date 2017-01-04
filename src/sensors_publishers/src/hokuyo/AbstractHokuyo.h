#ifndef PROJECT_ABSTRACTHOKUYO_H
#define PROJECT_ABSTRACTHOKUYO_H

#include "std_msgs/Int32MultiArray.h"

using namespace std_msgs;

class AbstractHokuyo {
public:

    virtual void init() = 0;

    virtual void readData() = 0;

    virtual Int32MultiArray getData() = 0;
};


#endif //PROJECT_ABSTRACTHOKUYO_H
