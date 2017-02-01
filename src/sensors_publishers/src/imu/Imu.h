#ifndef PROJECT_IMU_H
#define PROJECT_IMU_H

#include "AbstractImu.h"

#include"ros/ros.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#define COMPASS_ALIGNMENT 900
#define dev_name "/dev/imu"

class Imu : public AbstractImu {
    int imu;
    sensor_msgs::Imu result;
public:

    void init();

    void readData();

    sensor_msgs::Imu getData();
};


#endif //PROJECT_IMU_H
