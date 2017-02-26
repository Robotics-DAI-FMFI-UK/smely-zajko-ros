#include "ros/ros.h"
#include "robot/AbstractRobot.h"
#include "robot/Robot.h"
#include "message_types/SbotMsg.h"

AbstractRobot *robot;

void sbotCallback(const message_types::SbotMsg &sbot) {
    std::cout << sbot.lstep << '\n';
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "ros_control");
    ros::NodeHandle nh;
    ros::Subscriber sbot_subscriber = nh.subscribe("sbot_publisher", 100, sbotCallback);

    robot = new Robot();

    ros::spin();

    return 0;
}