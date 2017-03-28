#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include "robot/AbstractRobot.h"
#include "robot/Robot.h"
#include "message_types/SbotMsg.h"
#include "message_types/GpsAngles.h"

AbstractRobot *robot;

message_types::SbotMsg sbot_msg;
message_types::GpsAngles gps_msg;
sensor_msgs::Imu imu_msg;
cv::Mat image;
std_msgs::Int32MultiArray hokuyo_msg;

int direction = 0;

void sbotCallback(const message_types::SbotMsg &msg) {
    ROS_ERROR("%f", msg.lstep);
}

void localizationAndPlanningCallback(const message_types::GpsAngles &msg) {
    gps_msg = msg;
}

void imuCallback(const sensor_msgs::Imu &msg) {
    ROS_INFO("x: [%f]", msg.orientation.x);
    imu_msg = msg;
}


void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgra8'.", msg->encoding.c_str());
    }
}

double predicted_dir = 0.0;
double running_mean = 0.0;
double running_mean_weight = 0.7;

void hokuyoAlgoCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    double max = -INFINITY;
    int max_index = 0;
    int i = 0;
    for (std::vector<double>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it) {
        if (*it > max) {
            max = *it;
            max_index = i;
        }
        i++;
    }
    predicted_dir = (running_mean * running_mean_weight) + (max_index * (1 - running_mean_weight));
    running_mean = (running_mean * 3.0 + predicted_dir) / 4.0;

    direction = running_mean;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "ros_control");
    ros::NodeHandle nh;
    ros::Subscriber sbot_subscriber = nh.subscribe("/sensors/sbot_publisher", 100, sbotCallback);
    ros::Subscriber localization_and_planning_subscriber = nh.subscribe("localization_and_planning", 100,
                                                                        localizationAndPlanningCallback);
    ros::Subscriber hokuyo_algo_subscriber = nh.subscribe("houyo_algo", 100, hokuyoAlgoCallback);
    ros::Subscriber imu_subscriber = nh.subscribe("/sensors/imu_publisher", 100, imuCallback);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/sensors/camera/image", 1, imageCallback);

    robot = new Robot();

    ros::Rate loop_rate(20);

    while (ros::ok()) {
        if (robot != NULL) {
            robot->set_speed(5);
            robot->set_direction(direction);
            ROS_ERROR("my direction is %lf", direction);
        }
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}