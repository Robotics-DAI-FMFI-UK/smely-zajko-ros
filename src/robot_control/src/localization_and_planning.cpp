#include "ros/ros.h"
#include "localizationAndPlanning/LocalizationAndPlanning.h"
#include "message_types/GpsAngles.h"

ros::Publisher pubPtr;
LocalizationAndPlanning *localizationAndPlanning = new LocalizationAndPlanning(500, 500);

void gpsCallback(const sensor_msgs::NavSatFix &gps) {
    pubPtr.publish(localizationAndPlanning->update(gps));

    cvShowImage("loc and planning", localizationAndPlanning->getGui());
    cv::waitKey(30);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "ros_control");
    ros::NodeHandle nh;
    ros::Subscriber gps_subscriber = nh.subscribe("/sensors/gps_publisher", 100, gpsCallback);
    pubPtr = nh.advertise<message_types::GpsAngles>("localization_and_planning", 100);

    localizationAndPlanning->readMap((char *) "/home/jozef/Desktop/smely-zajko/maps/wien.osm");

    ros::spin();

    return 0;
}