#ifndef PROJECT_HOKUYOFILES_H
#define PROJECT_HOKUYOFILES_H

#include "AbstractHokuyo.h"

#include <dirent.h>
#include <cstdio>
#include <fstream>

class HokuyoFiles : public AbstractHokuyo {
    Int32MultiArray data;
    DIR *dir;
    struct dirent *ent;
public:

    void init();

    void readData();

    Int32MultiArray getData();
};


#endif //PROJECT_HOKUYOFILES_H
