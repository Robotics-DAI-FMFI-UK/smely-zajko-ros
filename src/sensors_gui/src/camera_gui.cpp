#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64.h>

double direction;
IplImage *direction_image;

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        cv::imshow("camera_gui", cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGRA8)->image);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgra8'.", msg->encoding.c_str());
    }
}

void directionCallback(const std_msgs::Float64ConstPtr &dir) {
    direction = dir->data;
}

void predictionCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        cv::Mat image;
        cv::resize(cv_bridge::toCvShare(msg)->image, image, cv::Size(640, 480));
        cvSet(direction_image, CV_RGB(255, 255, 255));
        cvLine(direction_image, cvPoint(80, 30), cvPoint((direction + 40) * 2 , 15),
               cvScalar(0, 0, 1), 2);
        cv::imshow("prediction_gui", image);
        cvShowImage("direction", direction_image);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgra8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "camera_gui");
    ros::NodeHandle nh;

    cv::namedWindow("camera_gui");
    cv::namedWindow("prediction_gui");
    cv::namedWindow("direction");

    direction_image = cvCreateImage(cvSize(160, 30), 32, 3);

    cv::startWindowThread();

    image_transport::ImageTransport it(nh);
//    image_transport::Subscriber sub = it.subscribe("/sensors/camera/image", 1, imageCallback);
    image_transport::Subscriber sub1 = it.subscribe("/control/camera_prediction", 1, predictionCallback);

    ros::Subscriber directionSubscriber = nh.subscribe("/control/directionPublisher", 1, directionCallback);
    ros::spin();

    cv::destroyWindow("camera_gui");
    cv::destroyWindow("prediction_gui");
    cv::destroyWindow("direction");

    return 0;
}