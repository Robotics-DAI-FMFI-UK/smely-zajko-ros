#include "ros/ros.h"
#include "localizationAndPlanning/LocalizationAndPlanning.h"
#include "message_types/GpsAngles.h"
#include "message_types/SbotMsg.h"

ros::Publisher pubPtr;
LocalizationAndPlanning *localizationAndPlanning = new LocalizationAndPlanning(500, 500);

message_types::SbotMsg sbot_msg;

sensor_msgs::NavSatFix loadingPoint = sensor_msgs::NavSatFix();
sensor_msgs::NavSatFix unloadingPoint = sensor_msgs::NavSatFix();
sensor_msgs::NavSatFix destinationPoint = sensor_msgs::NavSatFix();

int headingState;

const int HEADING_LOADING = 0;
const int LOADING = 1;
const int HEADING_UNLOADING = 2;
const int UNLOADING = 3;
const int HEADING_DEST = 4;
const int END = 5;

const float FULL = 1;
const float EMPTY = 0;

int debug_payload = 0;

void sbotCallback(const message_types::SbotMsg &msg) {
    sbot_msg = msg;
}

void setState(int state) {
    headingState = state;
}

void gpsCallback(const sensor_msgs::NavSatFix &gps) {
    message_types::GpsAngles actualHeading = localizationAndPlanning->update(gps);

    if (actualHeading.map == DBL_MAX) {
        if (headingState == HEADING_LOADING) {
            printf("LOADING\n");
            system("echo LOADING | espeak");
            setState(LOADING);
        } else if (headingState == LOADING && sbot_msg.payload == FULL) {
            printf("HEADING_UNLOADING\n");
            system("echo 'HEADING UNLOADING' | espeak");
            localizationAndPlanning->setDestination(unloadingPoint);
            setState(HEADING_UNLOADING);
        } else if (headingState == HEADING_UNLOADING) {
            printf("UNLOADING\n");
            system("echo 'UNLOADING' | espeak");
            setState(UNLOADING);
        } else if (headingState == UNLOADING && sbot_msg.payload == EMPTY) {
            printf("HEADING_DEST\n");
            system("echo 'HEADING DESTINATION' | espeak");
            localizationAndPlanning->setDestination(destinationPoint);
            setState(HEADING_DEST);
        } else if (headingState == HEADING_DEST) {
            printf("FINISH\n");
            setState(END);
            system("echo 'FINISH' | espeak");
        }
    }

    actualHeading.headingState = headingState;

    pubPtr.publish(actualHeading);

    cvShowImage("loc and planning", localizationAndPlanning->getGui());
    cv::waitKey(30);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "ros_control");
    ros::NodeHandle nh;
    ros::Subscriber gps_subscriber = nh.subscribe("/sensors/gps_publisher", 10, gpsCallback);

    ros::Subscriber sbot_subscriber = nh.subscribe("/sensors/sbot_publisher", 10, sbotCallback);

    pubPtr = nh.advertise<message_types::GpsAngles>("localization_and_planning", 10);

    localizationAndPlanning->readMap((char *) "/home/zajko/Projects/smely-zajko-ros/resources/maps/zilina.osm");

    loadingPoint.latitude = 49.2116839;
    loadingPoint.longitude = 18.7451427;

    unloadingPoint.latitude = 49.2118122;
    unloadingPoint.longitude = 18.7452533;

    destinationPoint.latitude = 49.2121765;
    destinationPoint.longitude = 18.7453255;

    localizationAndPlanning->setDestination(loadingPoint);

    headingState = HEADING_LOADING;

    ros::spin();

    return 0;
}
