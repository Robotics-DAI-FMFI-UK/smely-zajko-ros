#ifndef PROJECT_GPS_H
#define PROJECT_GPS_H

#include "AbstractGps.h"

#include "ros/ros.h"
#include <fcntl.h>
#include <termios.h>
#include <vector>

#define dev_name "/dev/galileo"

using namespace std;

class Gps : public AbstractGps {
    int gps;
    int bufp;

    char b[1024];
    char b2[1024];

    sensor_msgs::NavSatFix parseLine(const char *s);

public:

    void init();

    void readData();

    sensor_msgs::NavSatFix getData();
};


#endif //PROJECT_GPS_H
