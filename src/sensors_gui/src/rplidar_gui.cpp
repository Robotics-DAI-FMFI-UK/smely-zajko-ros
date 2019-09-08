#include <vector>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include "message_types/RpLidarObstacle.h"
#include <cmath>


cv::Mat result;
int rays = 0;
double angles[400];
double distances[400];

int guiWidth = 320;
int guiHeight = 320;

double maxDist = 0;

void rplidarCallback(const message_types::RpLidarObstacle &msg) {
    rays = msg.rays;
    maxDist = 0;
    for(int i = 0; i < rays; i++) {
        angles[i] = msg.angles.data[i];
        distances[i] = msg.distances.data[i];
        if (distances[i] > maxDist) maxDist = distances[i];
    }
}

void render_window() {
    result = cv::Mat(guiWidth, guiHeight, CV_8UC3, cv::Scalar(255, 255, 255));

    cv::Point center(160, 160);

    double ratio = 150.0 / maxDist;

    for (int i = 0; i < rays; i++) {
        double angle = angles[i];
        double dist = distances[i] * ratio;

        cv::Point end = center + cv::Point(dist * sin(angle), -dist * cos(angle));
        cv::line(result, center, end, cv::Scalar(0, 0, 0));
    }


    cv::imshow("rplidar", result);
    cv::waitKey(1);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "rplidar_gui");
    ros::NodeHandle n;

//    cv::Mat empty_frame = cv::Mat::zeros(guiWidth, guiHeight, CV_8UC3);
//    cv::namedWindow("rplidar", CV_WINDOW_AUTOSIZE);
//    cv::imshow("rplidar", empty_frame);
//    cv::waitKey(30);

    ros::Subscriber rplidar_subscriber = n.subscribe("/sensors/rplidar_publisher", 2, rplidarCallback);

    ros::Rate loop_rate(30);

    while (ros::ok()) {
        render_window();

        ros::spinOnce();

        loop_rate.sleep();
    }

    cvDestroyWindow("rplidar");

    return 0;
}
