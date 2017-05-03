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
    ros::Subscriber gps_subscriber = nh.subscribe("/sensors/gps_publisher", 10, gpsCallback);
    pubPtr = nh.advertise<message_types::GpsAngles>("localization_and_planning", 10);

    localizationAndPlanning->readMap((char *) "/home/zajko/Projects/smely-zajko-ros/resources/maps/botanicka.osm");

    sensor_msgs::NavSatFix destinationPoint = sensor_msgs::NavSatFix();
    destinationPoint.latitude = 48.1451892;
    destinationPoint.longitude = 17.0737996;

    localizationAndPlanning->setDestination(destinationPoint);

    ros::spin();

    return 0;
}