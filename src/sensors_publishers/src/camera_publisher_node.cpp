#include "ros/ros.h"
#include "camera/AbstractCamera.h"
#include "camera/Camera.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "camera_publisher");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    cv::waitKey(30);

    cv::Mat image;

    AbstractCamera *camera = new Camera();
    camera->init();

    sensor_msgs::ImagePtr msg;
    while (nh.ok()) {

        camera->readData();
        image = camera->getData();

        if (!image.empty()) {
            msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, image).toImageMsg();
            pub.publish(msg);
            cv::waitKey(30);
        }

        ros::spinOnce();
    }

    return 0;
}