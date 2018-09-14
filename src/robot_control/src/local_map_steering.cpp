#include <stdlib.h>
#include "ros/ros.h"
#include "localMap/LocalMap.h"
#include "message_types/SbotMsg.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include <std_msgs/Float64.h>
#include "message_types/GpsAngles.h"
#include "sensor_msgs/Imu.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "string.h"

ros::Publisher steeringPublisher;

bool sbot = false, hokuyo = false;

LocalMap* localMap;

double d2r(double d) {
    return d * (pi / 180);
}

void say(const char *msg) {
    char out[256];
    snprintf(out, 255, "echo \"%s\" | espeak -a 200 -p 20 -s 80", msg);
    system(out);
}


void hokuyoCallback(const std_msgs::Int32MultiArray::ConstPtr &array) {
    int i = 0;
    int data[1081];
    for (std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
        data[i] = *it;
        i++;
    }
    localMap->setHokuyoData(data);

}

void sbotCallback(const message_types::SbotMsg &msg) {
    localMap->updateRobotPosition((long) msg.lstep, (long) msg.rstep);
}

void globalMapCallback(const message_types::GpsAngles &msg) {
    localMap->setGlobalMapData(d2r(msg.currWayHeading), d2r(msg.nextWayHeading), msg.distToWayEnd);
}

void imuCallback(const sensor_msgs::Imu &msg) {
    localMap->setCompassHeading(d2r(msg.orientation.x / 10));
}

void cameraCallback(const std_msgs::UInt8MultiArray::ConstPtr &array) {
    unsigned char data[3600];
    int i = 0;
    for (std::vector<unsigned char>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
        data[i] = *it;
        i++;
    }
    localMap->setImageData(data);
}

int c = 0;

void drawMap(const ros::TimerEvent&) {
    if (c > 1) {
        localMap->doUpdate();
        c = 0;
    }
    c++;

    cv::imshow("local map", localMap->getGui());
    cv::waitKey(1);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "ros_control");
    ros::NodeHandle nh;

    ros::Subscriber sbot_subscriber = nh.subscribe("/control/base_data", 10, sbotCallback);
    ros::Subscriber hokuyo_subscriber = nh.subscribe("/sensors/hokuyo_publisher", 10, hokuyoCallback);
    ros::Subscriber global_map_subscriber = nh.subscribe("/control/localization_and_planning", 10, globalMapCallback);
    ros::Subscriber imu_subscriber = nh.subscribe("/sensors/imu_publisher", 10, imuCallback);
    ros::Subscriber camera_subscriber = nh.subscribe("/sensors/camera/evaluated_image", 10, cameraCallback);

    ros::Publisher heading_publisher = nh.advertise<std_msgs::Float64>("/control/local_map", 10);

    localMap = new LocalMap(600, 600, heading_publisher);

    ros::Timer draw = nh.createTimer(ros::Duration(1), drawMap);

//    cv::imshow("local map", localMap->getGui());
//    cv::waitKey(0);

    ros::spin();

    return 0;
}
