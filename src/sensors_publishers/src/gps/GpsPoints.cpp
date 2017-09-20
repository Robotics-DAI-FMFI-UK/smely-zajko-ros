#include "GpsPoints.h"

void GpsPoints::init() {
}

void GpsPoints::readData() {
}

sensor_msgs::NavSatFix GpsPoints::getData() {
    sensor_msgs::NavSatFix fix;

    fix.latitude = 49.2116839;
    fix.longitude = 18.7451427;

    return fix;
}