#ifndef PROJECT_GPSFILES_H
#define PROJECT_GPSFILES_H


#include "AbstractGps.h"
#include <dirent.h>
#include <cstdio>
#include <fstream>

class GpsFiles : public AbstractGps {
    sensor_msgs::NavSatFix data;
    std::fstream file;
public:

    void init();

    void readData();

    sensor_msgs::NavSatFix getData();
};


#endif //PROJECT_GPSFILES_H
