#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &gps) {
    std::cout << gps->longitude << " " << gps->latitude << '\n';
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "gps_subscriber");
    ros::NodeHandle n;
    ros::Subscriber subscriber = n.subscribe("gps_publisher", 100, gpsCallback);

    ros::spin();

    return 0;
}