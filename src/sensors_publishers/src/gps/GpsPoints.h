#ifndef PROJECT_GPS_POINTS_H
#define PROJECT_GPS_POINTS_H

#include "AbstractGps.h"

#include "ros/ros.h"
#include <fcntl.h>
#include <termios.h>
#include <vector>

#define dev_name "/dev/galileo"

using namespace std;

class GpsPoints : public AbstractGps {

public:

    void init();

    void readData();

    sensor_msgs::NavSatFix getData();
};


#endif //PROJECT_GPS_POINTS_H
