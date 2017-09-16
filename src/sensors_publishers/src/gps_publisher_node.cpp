#include "ros/ros.h"
#include "gps/AbstractGps.h"
#include "gps/Gps.h"
#include "gps/GpsFiles.h"
#include "gps/GpsPoints.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_publisher");
    ros::NodeHandle n;
    ros::Publisher gps_publisher = n.advertise<sensor_msgs::NavSatFix>("gps_publisher", 10);

    AbstractGps *gps = new Gps();
    gps->init();

    ros::Rate loop_rate(3);

    ros::Duration(2).sleep();

    while (ros::ok()) {
        if (gps_publisher.getNumSubscribers() > 0) {
            gps->readData();

            sensor_msgs::NavSatFix g = gps->getData();
            if (g.latitude) {
                gps_publisher.publish(g);
            }
            ros::spinOnce();

            loop_rate.sleep();
        }
    }

    return 0;
}