#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>
#include "robot/AbstractRobot.h"
#include "robot/Robot.h"
#include "message_types/SbotMsg.h"
#include "message_types/GpsAngles.h"

AbstractRobot *robot;

void sbotCallback(const message_types::SbotMsg &sbot) {
    ROS_ERROR("%f", sbot.lstep);
}

void localizationAndPlanningCallback(const message_types::GpsAngles &msg) {

}

void imuCallback(const sensor_msgs::Imu::ConstPtr &imu) {
    ROS_INFO("x: [%f]", imu->orientation.x);
}


void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgra8'.", msg->encoding.c_str());
    }
}

void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr &array) {

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "ros_control");
    ros::NodeHandle nh;
    ros::Subscriber sbot_subscriber = nh.subscribe("/sensors/sbot_publisher", 100, sbotCallback);
    ros::Subscriber localization_and_planning_subscriber = nh.subscribe("localization_and_planning", 100,
                                                                        localizationAndPlanningCallback);
    ros::Subscriber imu_subscriber = nh.subscribe("/sensors/imu_publisher", 100, imuCallback);
    ros::Subscriber hokuyo_subscriber = nh.subscribe("/sensors/hokuyo_publisher", 100, arrayCallback);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/sensors/camera/image", 1, imageCallback);

    robot = new Robot();

    ros::spin();

    return 0;
}