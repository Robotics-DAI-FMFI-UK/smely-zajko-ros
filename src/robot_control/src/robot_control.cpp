#include "ros/ros.h"
#include "robot/AbstractRobot.h"
#include "robot/Robot.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "ros_control");
    ros::NodeHandle nh;

    AbstractRobot *robot = new Robot();

    return 0;
}