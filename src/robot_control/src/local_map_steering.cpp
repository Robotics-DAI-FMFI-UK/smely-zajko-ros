#include <stdlib.h>
#include "ros/ros.h"
#include "localMap/LocalMap.h"
#include "message_types/SbotMsg.h"
#include "message_types/RpLidarObstacle.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include <std_msgs/Float64.h>
#include "message_types/GpsAngles.h"
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "string.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define RAD2DEG 57.295779513

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


int data[1081];
void hokuyoCallback(const std_msgs::Int32MultiArray::ConstPtr &array) {
    int i = 0;
    for (std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
        data[i] = *it;
        i++;
    }
    localMap->setHokuyoData(data);
}

double angles[400];
double distances[400];
void rplidarCallback(const message_types::RpLidarObstacle &msg) {
    int rays = msg.rays;
    for(int i = 0; i < rays; i++) {
        distances[i] = msg.distances.data[i];
        angles[i] = msg.angles.data[i];
    }
    localMap->setRpLidarData(rays, distances, angles);
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

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // Camera position in map frame
    double tx = msg->pose.position.x;
    double ty = msg->pose.position.y;
    double tz = msg->pose.position.z;

    // Orientation quaternion
    tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
//    ROS_INFO("Received odom in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
//             msg->header.frame_id.c_str(),
//             tx, ty, tz,
//             roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
//    ROS_INFO("X: %.2fcm Y: %.2fcm A: %.2f°", tx*100, ty*100, yaw * RAD2DEG);

    // coordinate system in local map is stupid (left handed with X left, Y back and Z down)
    // zed returns pose in not-stupid system (right handed with X forward, Y left and Z up)
    // need to flip axes and directions to make it work
    localMap->setPose(-ty*100, tx*100, -yaw);
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

    ros::Subscriber sbot_subscriber = nh.subscribe("/control/base_data", 2, sbotCallback);
    ros::Subscriber hokuyo_subscriber = nh.subscribe("/sensors/hokuyo_publisher", 2, hokuyoCallback);
    ros::Subscriber rplidar_subscriber = nh.subscribe("/sensors/rplidar_publisher", 2, rplidarCallback);
    ros::Subscriber global_map_subscriber = nh.subscribe("/control/localization_and_planning", 2, globalMapCallback);
    ros::Subscriber imu_subscriber = nh.subscribe("/sensors/imu_publisher", 2, imuCallback);
    ros::Subscriber camera_subscriber = nh.subscribe("/sensors/camera/evaluated_image", 2, cameraCallback);
//    ros::Subscriber zed_pose_subscriber = nh.subscribe("/pose", 2, poseCallback);

    ros::Publisher heading_publisher = nh.advertise<std_msgs::Float64>("/control/local_map", 10);

    localMap = new LocalMap(600, 600, heading_publisher);

    ros::Timer draw = nh.createTimer(ros::Duration(0.5), drawMap);

//    cv::imshow("local map", localMap->getGui());//    ROS_INFO("Received odom in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
//             msg->header.frame_id.c_str(),
//             tx, ty, tz,
//             roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);

//    cv::waitKey(0);

    ros::spin();

    return 0;
}
