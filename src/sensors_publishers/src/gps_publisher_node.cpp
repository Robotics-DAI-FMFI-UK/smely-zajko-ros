#include "ros/ros.h"
#include "gps/AbstractGps.h"
#include "gps/Gps.h"
#include "gps/GpsFiles.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_publisher");
    ros::NodeHandle n;
    ros::Publisher gps_publisher = n.advertise<sensor_msgs::NavSatFix>("gps_publisher", 10);

    AbstractGps *gps = new Gps();
    gps->init();

    ros::Rate loop_rate(30);

    ros::Duration(0.5).sleep();

    while (ros::ok()) {
        if (gps_publisher.getNumSubscribers() > 0) {
            gps->readData();

            gps_publisher.publish(gps->getData());

            ros::spinOnce();

            loop_rate.sleep();
        }
    }

    return 0;
}