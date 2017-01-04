#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "camera_publisher");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    cv::waitKey(30);

    cv::VideoCapture cap(0); // open the default camera
    if (!cap.isOpened()) {
        ROS_ERROR("Camera is not initialized");

        return -1;
    }

    cv::Mat image;
    sensor_msgs::ImagePtr msg;
    while (nh.ok()) {

        cv::Mat frame;
        cap >> frame;
        cv::cvtColor(frame, image, CV_BGR2BGRA);
        // detect edges in image, maybe usable...
        /*cv::GaussianBlur(image, image, cv::Size(7, 7), 1.5, 1.5);
        cv::Canny(image, image, 0, 30, 3);*/

        if (!frame.empty()) {
            cv::flip(image, image, 1);
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            pub.publish(msg);
            cv::waitKey(30);
        }

        ros::spinOnce();
    }

    return 0;
}