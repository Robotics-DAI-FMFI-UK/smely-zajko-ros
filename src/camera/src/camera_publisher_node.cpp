#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    cv::Mat image = cv::imread("/home/jozef/Desktop/smely-zajko/example_input.jpg", CV_LOAD_IMAGE_COLOR);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    ros::Rate loop_rate(5);
    cv::waitKey(30);
    while (nh.ok()) {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}