#include"ros/ros.h"
#include "sensor_msgs/Imu.h"

#include "imu/AbstractImu.h"
#include "imu/Imu.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_publisher");
    ros::NodeHandle nh;
    ros::Publisher imu_publisher = nh.advertise<sensor_msgs::Imu>("imu_publisher", 100);

    AbstractImu *imu = new Imu();
    imu->init();

    ros::Rate loop_rate(10);

    ros::Duration(0.5).sleep();

    while (ros::ok()) {
        if (imu_publisher.getNumSubscribers() > 0) {
            imu->readData();

            imu_publisher.publish(imu->getData());

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    return 0;
}