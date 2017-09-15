#include "ros/ros.h"
#include "localizationAndPlanning/LocalizationAndPlanning.h"
#include "message_types/GpsAngles.h"
#include "message_types/HeadingState.h"

ros::Publisher pubPtr;
LocalizationAndPlanning *localizationAndPlanning = new LocalizationAndPlanning(500, 500);

sensor_msgs::NavSatFix loadingPoint = sensor_msgs::NavSatFix();
sensor_msgs::NavSatFix unloadingPoint = sensor_msgs::NavSatFix();
sensor_msgs::NavSatFix destinationPoint = sensor_msgs::NavSatFix();

message_types::HeadingState state_msg;

const int HEADING_LOADING = 0;
const int LOADING = 1;
const int HEADING_UNLOADING = 2;
const int UNLOADING = 3;
const int HEADING_DEST = 4;

void gpsCallback(const sensor_msgs::NavSatFix &gps) {
    pubPtr.publish(localizationAndPlanning->update(gps));

    cvShowImage("loc and planning", localizationAndPlanning->getGui());
    cv::waitKey(30);
}

void headingStateCallback(const message_types::HeadingState &state){
    state_msg = state;
    ROS_INFO("---------------------------- HEADING STATE %d ------------------------------------------", state_msg.headingState);
    if (state_msg.headingState == LOADING) {
        localizationAndPlanning->setDestination(unloadingPoint);
        system("echo heading unloading | espeak");
    } else if (state_msg.headingState == UNLOADING) {
        localizationAndPlanning->setDestination(destinationPoint);
        system("echo heading destination | espeak");
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "ros_control");
    ros::NodeHandle nh;
    ros::Subscriber gps_subscriber = nh.subscribe("/sensors/gps_publisher", 10, gpsCallback);

    ros::Subscriber heading_state_subscriber = nh.subscribe("statePublisher", 10, headingStateCallback);

    pubPtr = nh.advertise<message_types::GpsAngles>("localization_and_planning", 10);

    localizationAndPlanning->readMap((char *) "/home/zajko/Projects/smely-zajko-ros/resources/maps/matfyz.osm");

    loadingPoint.latitude = 48.1527956;
    loadingPoint.longitude = 17.0711376;

    unloadingPoint.latitude = 49.1527956;
    unloadingPoint.longitude = 17.0711376;

    destinationPoint.latitude = 47.1527956;
    destinationPoint.longitude = 17.0711376;

    localizationAndPlanning->setDestination(loadingPoint);

    ros::spin();

    /*ros::Rate r(1.0 / 10.0);

	gpsCallback(loadingPoint);
	ros::spinOnce();
	r.sleep();
	gpsCallback(unloadingPoint);
	ros::spinOnce();
	r.sleep();
	gpsCallback(destinationPoint);
	ros::spinOnce();
	r.sleep();*/
    return 0;
}
