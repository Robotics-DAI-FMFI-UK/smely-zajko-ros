#ifndef PROJECT_HOKUYOFILE_H
#define PROJECT_HOKUYOFILE_H

#include "AbstractHokuyo.h"

class HokuyoFile : public AbstractHokuyo {
public:
    void init();

    void readData();

    Int32MultiArray getData();
};


#endif //PROJECT_HOKUYOFILE_H
