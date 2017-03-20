#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        cv::imshow("camera_gui", cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGRA8)->image);
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
    cv::startWindowThread();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/sensors/camera/image", 1, imageCallback);

    ros::spin();

    cv::destroyWindow("view");

    return 0;
}