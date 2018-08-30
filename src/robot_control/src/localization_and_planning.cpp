#include <stdlib.h>
#include "ros/ros.h"
#include "localizationAndPlanning/LocalizationAndPlanning.h"
#include "message_types/GpsAngles.h"
#include "message_types/SbotMsg.h"
#include "string.h"

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

void say(const char *msg) {
    char out[256];
    snprintf(out, 255, "echo \"%s\" | espeak -a 200 -p 20 -s 80", msg);
    system(out);
}

void gpsCallback(const sensor_msgs::NavSatFix &gps) {
    message_types::GpsAngles actualHeading = localizationAndPlanning->update(gps);

    if (actualHeading.map == DBL_MAX) {
        if (headingState == HEADING_LOADING) {
            printf("LOADING\n");
            say("LOADING");

            setState(LOADING);
        } else if (headingState == LOADING && sbot_msg.payload == FULL) {
            printf("HEADING_UNLOADING\n");
            say("HEADING UNLOADING");

            localizationAndPlanning->setDestination(unloadingPoint);
            setState(HEADING_UNLOADING);
        } else if (headingState == HEADING_UNLOADING) {
            printf("UNLOADING\n");
            say("UNLOADING");

            setState(UNLOADING);
        } else if (headingState == UNLOADING && sbot_msg.payload == EMPTY) {
            printf("HEADING_DEST\n");
            say("HEADING DESTINATION");

            localizationAndPlanning->setDestination(destinationPoint);
            setState(HEADING_DEST);
        } else if (headingState == HEADING_DEST) {
            printf("FINISH\n");
            setState(END);
            say("FINISH");
        }
    }

    actualHeading.headingState = headingState;

    printf("Distance: %f\n", localizationAndPlanning->distance(localizationAndPlanning->destinationPoint,
                                                             localizationAndPlanning->curPoint) * 1000);
    pubPtr.publish(actualHeading);

    cvShowImage("loc and planning", localizationAndPlanning->getGui());
    cv::waitKey(1);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "ros_control");
    ros::NodeHandle nh;
    ros::Subscriber gps_subscriber = nh.subscribe("/sensors/gps_publisher", 1, gpsCallback);

    ros::Subscriber sbot_subscriber = nh.subscribe("/sensors/sbot_publisher", 10, sbotCallback);

    pubPtr = nh.advertise<message_types::GpsAngles>("localization_and_planning", 10);

    //localizationAndPlanning->readMap((char *) "/home/zajko/Projects/smely-zajko-ros/resources/maps/zilina.osm");
    //localizationAndPlanning->readMap((char *) "/home/zajko/Projects/smely-zajko-ros/resources/maps/botanicka.osm");
    localizationAndPlanning->readMap((char *) "/home/zajko/Projects/smely-zajko-ros/resources/maps/homologacie_fei.osm");

/*    loadingPoint.latitude = 49.2129610;
    loadingPoint.longitude = 18.7447602;

    unloadingPoint.latitude = 49.2126778;
    unloadingPoint.longitude = 18.7439919;

    destinationPoint.latitude = 49.2128361;
    destinationPoint.longitude = 18.7446799;
*/

/* homologacia fei: */
 
    loadingPoint.latitude = 48.15143;
    loadingPoint.longitude = 17.0729;

    unloadingPoint.latitude = 48.15159;
    unloadingPoint.longitude = 17.07298;

    destinationPoint.latitude = 48.1517;
    destinationPoint.longitude = 17.07329;

    /*
    loadingPoint.latitude = 48.1457841;
    loadingPoint.longitude = 17.0740046;


    unloadingPoint.latitude = 48.1458947;
    unloadingPoint.longitude = 17.0722153;


    destinationPoint.latitude = 48.1473408;
    destinationPoint.longitude = 17.0725359;
    */
    localizationAndPlanning->setDestination(loadingPoint);

    headingState = HEADING_LOADING;

    ros::spin();

    return 0;
}
