#include <stdlib.h>
#include "ros/ros.h"
#include "localizationAndPlanning/LocalizationAndPlanning.h"
#include "message_types/GpsAngles.h"
#include "message_types/SbotMsg.h"
#include <std_msgs/UInt8.h>
#include "string.h"

int cameraAction;

const int DETECT_ROAD = 0;
const int DETECT_QR = 1;

ros::Publisher cameraActionPublisher;

ros::Publisher locPublisher;
LocalizationAndPlanning *localizationAndPlanning = new LocalizationAndPlanning(500, 500);

message_types::SbotMsg sbot_msg;

sensor_msgs::NavSatFix loadingPoint = sensor_msgs::NavSatFix();
sensor_msgs::NavSatFix unloadingPoint = sensor_msgs::NavSatFix();
sensor_msgs::NavSatFix destinationPoint = sensor_msgs::NavSatFix();

sensor_msgs::NavSatFix newTarget = sensor_msgs::NavSatFix();
bool targetValid = false;

int headingState;

const int START = 0;
const int HEADING_LOADING = 1;
const int LOADING = 2;
const int HEADING_UNLOADING = 3;
const int UNLOADING = 4;
const int HEADING_DEST = 5;
const int END = 6;

const float FULL = 1;
const float EMPTY = 0;

int debug_payload = 0;

void sbotCallback(const message_types::SbotMsg &msg) {
    sbot_msg = msg;
}

void targetCallback(const sensor_msgs::NavSatFix &dest) {
    newTarget = dest;
    targetValid = true;
    printf("new target from QR: %f %f\n", dest.latitude, dest.longitude);
}

void resetTarget() {
    targetValid = false;
}

void setState(int state) {
    headingState = state;
}

void setCameraAction(int action) {
    cameraAction = action;
}

void say(const char *msg) {
    char out[256];
    snprintf(out, 255, "echo \"%s\" | espeak -a 200 -p 20 -s 80", msg);
    system(out);
}

int spamCounter = 0;

void gpsCallback(const sensor_msgs::NavSatFix &gps) {
    message_types::GpsAngles actualHeading = localizationAndPlanning->update(gps);
    std_msgs::UInt8 actionMsg;

    if (headingState == START) {
        if (!targetValid) {
            setCameraAction(DETECT_QR);
            if (spamCounter == 0) say("WAITING FOR QR CODE");
            spamCounter++;
            if (spamCounter > 10) spamCounter = 0;
        } else {
            say("HEADING LOADING");
            localizationAndPlanning->setDestination(newTarget);
            setState(HEADING_LOADING);
            setCameraAction(DETECT_ROAD);
        }
    }
    if (actualHeading.map == DBL_MAX) {
        if (headingState == HEADING_LOADING) {
            printf("LOADING\n");
            say("LOADING");

            setState(LOADING);
            resetTarget();
            setCameraAction(DETECT_QR);
        } else if (headingState == LOADING && sbot_msg.payload == FULL && targetValid) {
            printf("HEADING_UNLOADING\n");
            say("HEADING UNLOADING");

            localizationAndPlanning->setDestination(newTarget);
            setState(HEADING_UNLOADING);
            setCameraAction(DETECT_ROAD);
        } else if (headingState == HEADING_UNLOADING) {
            printf("UNLOADING\n");
            say("UNLOADING");

            setState(UNLOADING);
            resetTarget();
            setCameraAction(DETECT_QR);
        } else if (headingState == UNLOADING && sbot_msg.payload == EMPTY) {
            printf("HEADING_DEST\n");
            say("HEADING DESTINATION");

            localizationAndPlanning->setDestination(destinationPoint);
            setState(HEADING_DEST);
            setCameraAction(DETECT_ROAD);
        } else if (headingState == HEADING_DEST) {
            printf("FINISH\n");
            setState(END);
            say("FINISH");
        }
    }

    actualHeading.headingState = headingState;
    actionMsg.data = cameraAction;

//    printf("Distance: %f\n", localizationAndPlanning->distance(localizationAndPlanning->destinationPoint,
//                                                             localizationAndPlanning->curPoint) * 1000);
    locPublisher.publish(actualHeading);
    cameraActionPublisher.publish(actionMsg);

    cvShowImage("loc and planning", localizationAndPlanning->getGui());
    cv::waitKey(1);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "ros_control");
    ros::NodeHandle nh;
    ros::Subscriber gps_subscriber = nh.subscribe("/sensors/gps_publisher", 1, gpsCallback);

    ros::Subscriber sbot_subscriber = nh.subscribe("/control/base_data", 10, sbotCallback);

    ros::Subscriber target_subscriber = nh.subscribe("/control/camera_qr_target", 1, targetCallback);

    locPublisher = nh.advertise<message_types::GpsAngles>("localization_and_planning", 10);
    cameraActionPublisher = nh.advertise<std_msgs::UInt8>("/control/camera_action", 10);

    //localizationAndPlanning->readMap((char *) "/home/zajko/Projects/smely-zajko-ros/resources/maps/zilina.osm");
    localizationAndPlanning->readMap((char *) "/home/zajko/Projects/smely-zajko-ros/resources/maps/lednice.osm");
//    localizationAndPlanning->readMap((char *) "/home/zajko/Projects/smely-zajko-ros/resources/maps/botanicka.osm");
//    localizationAndPlanning->readMap((char *) "/home/zajko/Projects/smely-zajko-ros/resources/maps/homologacie_fei.osm");

//    loadingPoint.latitude = 49.2129610;
//    loadingPoint.longitude = 18.7447602;

/*    unloadingPoint.latitude = 49.2126778;
    unloadingPoint.longitude = 18.7439919;

    destinationPoint.latitude = 49.2128361;
    destinationPoint.longitude = 18.7446799;
*/

/* homologacia fei: */
 
//    loadingPoint.latitude = 48.15143;
//    loadingPoint.longitude = 17.0729;
//
//    unloadingPoint.latitude = 48.15159;
//    unloadingPoint.longitude = 17.07298;
//
//    destinationPoint.latitude = 48.1517;
//    destinationPoint.longitude = 17.07329;

    /*
    loadingPoint.latitude = 48.1457841;
    loadingPoint.longitude = 17.0740046;


    unloadingPoint.latitude = 48.1458947;
    unloadingPoint.longitude = 17.0722153;


    destinationPoint.latitude = 48.1473408;
    destinationPoint.longitude = 17.0725359;
    */

    // default bod aby LocalizationAndPlanning nerobilo chyby kym sa caka na QRkod
    loadingPoint.latitude = 48.14703;
    loadingPoint.longitude = 17.07314;
    localizationAndPlanning->setDestination(loadingPoint);

    destinationPoint.latitude = 48.14721;
    destinationPoint.longitude = 17.07296;

    headingState = START;

    say("WAITING FOR GPS");

    ros::spin();

    return 0;
}
