#include "ros/ros.h"
#include "robot/AbstractRobot.h"
#include "robot/Robot.h"
#include "message_types/SbotMsg.h"
#include "LocalizationAndPlanning.h"

AbstractRobot *robot;
LocalizationAndPlanning *localizationAndPlanning = new LocalizationAndPlanning(500, 500);


void sbotCallback(const message_types::SbotMsg &sbot) {
    std::cout << sbot.lstep << '\n';
}

void gpsCallback(const sensor_msgs::NavSatFix &gps) {
    localizationAndPlanning->update(gps);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "ros_control");
    ros::NodeHandle nh;
    ros::Subscriber sbot_subscriber = nh.subscribe("sbot_publisher", 100, sbotCallback);
    ros::Subscriber gps_subscriber = nh.subscribe("gps_publisher", 100, gpsCallback);

    localizationAndPlanning->readMap((char *) "/home/jozef/Desktop/smely-zajko/maps/wien.osm");

    cvShowImage("laser", localizationAndPlanning->getGui());
    cv::waitKey(30);
    robot = new Robot();

    ros::spin();

    return 0;
}