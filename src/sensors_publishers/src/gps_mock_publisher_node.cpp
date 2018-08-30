#include "ros/ros.h"
#include "gps/AbstractGps.h"
#include "gps/Gps.h"
#include "gps/GpsFiles.h"
#include "gps/GpsPoints.h"

// Mock GPS
// publish GPS coords read from a file

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_publisher");
    ros::NodeHandle n;
    ros::Publisher gps_publisher = n.advertise<sensor_msgs::NavSatFix>("gps_publisher", 1);

    ros::Rate loop_rate(1);

    ros::Duration(2).sleep();

    while (ros::ok()) {
        if (gps_publisher.getNumSubscribers() > 0) {

            ifstream is;
            is.open("/home/zajko/Projects/smely-zajko-ros/resources/gps_mock.txt");
            double lat, lon;
            char x;
            if (is) is >> lat >> x >> lon;
            is.close();
            sensor_msgs::NavSatFix g;
            g.latitude = lat;
            g.longitude = lon;

            gps_publisher.publish(g);

            ros::spinOnce();

            loop_rate.sleep();
        }
    }

    return 0;
}