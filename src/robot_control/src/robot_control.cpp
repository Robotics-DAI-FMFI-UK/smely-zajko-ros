#include <sensor_msgs/Imu.h>
#include "ros/ros.h"
#include "robot/AbstractRobot.h"
#include "robot/Robot.h"
#include "message_types/SbotMsg.h"
#include "message_types/GpsAngles.h"

AbstractRobot *robot;

void sbotCallback(const message_types::SbotMsg &sbot) {
    std::cout << sbot.lstep << '\n';
}

void localizationAndPlanningCallback(const message_types::GpsAngles &msg) {

}

void imuCallback(const sensor_msgs::Imu::ConstPtr &imu) {
    ROS_INFO("x: [%f]", imu->orientation.x);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "ros_control");
    ros::NodeHandle nh;
    ros::Subscriber sbot_subscriber = nh.subscribe("sbot_publisher", 100, sbotCallback);
    ros::Subscriber localization_and_planning_subscriber = nh.subscribe("localization_and_planning", 100,
                                                                        localizationAndPlanningCallback);
    ros::Subscriber imu_subscriber = nh.subscribe("imu_publisher", 100, imuCallback);

    robot = new Robot();

    ros::spin();

    return 0;
}