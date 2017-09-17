#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include "robot/AbstractRobot.h"
#include "robot/Robot.h"
#include "message_types/SbotMsg.h"
#include "message_types/GpsAngles.h"
#include "message_types/HeadingState.h"

message_types::SbotMsg sbot_msg;
message_types::GpsAngles gps_msg;

void sbotCallback(const message_types::SbotMsg &msg) {
    sbot_msg = msg;
}

void localizationAndPlanningCallback(const message_types::GpsAngles &msg) {
    gps_msg = msg;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "state_handler_manager_producer_rotk");
    ros::NodeHandle nh;

    ros::Subscriber localization_and_planning_subscriber = nh.subscribe("localization_and_planning", 10,
                                                                        localizationAndPlanningCallback);
    ros::Subscriber sbot_subscriber = nh.subscribe("/sensors/sbot_publisher", 10, sbotCallback);

    return 0;
}