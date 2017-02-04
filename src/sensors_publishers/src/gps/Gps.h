#ifndef PROJECT_GPS_H
#define PROJECT_GPS_H

#include "AbstractGps.h"

#include "ros/ros.h"
#include <fcntl.h>
#include <termios.h>

#define dev_name "/dev/galileo"

class Gps : public AbstractGps {
    int gps;
    int bufp;

    char b[1024];
    char b2[1024];
public:

    void init();

    void readData();

    sensor_msgs::NavSatFix getData();
};


#endif //PROJECT_GPS_H
