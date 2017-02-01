#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"


void imuCallback(const sensor_msgs::Imu::ConstPtr &imu) {
    ROS_INFO("x: [%f]", imu->orientation.x);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "imu_subscriber");
    ros::NodeHandle n;
    ros::Subscriber subscriber = n.subscribe("imu_publisher", 100, imuCallback);

    ros::spin();

    return 0;
}