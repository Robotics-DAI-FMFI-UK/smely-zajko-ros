#include "ros/ros.h"
#include "localizationAndPlanning/LocalizationAndPlanning.h"
#include "message_types/GpsAngles.h"
#include "yaml-cpp"

ros::Publisher pubPtr;
LocalizationAndPlanning *localizationAndPlanning = new LocalizationAndPlanning(500, 500);

sensor_msgs::NavSatFix loadingPoint = sensor_msgs::NavSatFix();
sensor_msgs::NavSatFix unloadingPoint = sensor_msgs::NavSatFix();
sensor_msgs::NavSatFix destinationPoint = sensor_msgs::NavSatFix();

message_types::HeadingState state_msg;

void handleArrival() {
    switch(state_msg) {
        case message_types::HeadingState::LOADING:
            localizationAndPlanning->setDestination(unloadingPoint);
            break;
        case message_types::HeadingState::UNLOADING:
            localizationAndPlanning->setDestination(destinationPoint);
            break;
    }
}

void gpsCallback(const sensor_msgs::NavSatFix &gps) {
    if (state_msg == message_types::HeadingState::LOADING) {
        localizationAndPlanning->setDestination(unloadingPoint);
    } else if (state_msg == message_types::HeadingState::UNLOADING) {
        localizationAndPlanning->setDestination(destinationPoint);
    }

    dst = localizationAndPlanning->update(gps)
    pubPtr.publish(localizationAndPlanning->update(dst));

    cvShowImage("loc and planning", localizationAndPlanning->getGui());
    cv::waitKey(30);
}

void headingStateCallback(const message_types::HeadingState &state){
    state_msg = msg;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "ros_control");
    ros::NodeHandle nh;
    ros::Subscriber gps_subscriber = nh.subscribe("/sensors/gps_publisher", 10, gpsCallback);
    ros::Subscriber heading_state_subscriber = nh.subscribe("robot_control", 10, headingStateCallback);
    pubPtr = nh.advertise<message_types::GpsAngles>("localization_and_planning", 10);

    localizationAndPlanning->readMap((char *) "/home/zajko/Projects/smely-zajko-ros/resources/maps/matfyz.osm");

    YAML::Node config = YAML::LoadFile("config.yaml");

    loadingPoint.latitude = config["loading_lat"].as<double>();
    loadingPoint.longitude = config["loading_lon"].as<double>();

    unloadingPoint.latitude = config["unloading_lat"].as<double>();
    unloadingPoint.longitude = config["unloading_lon"].as<double>();

    destinationPoint.latitude = config["destination_lat"].as<double>();
    destinationPoint.longitude = config["destination_lon"].as<double>();

    localizationAndPlanning->setDestination(loadingPoint);

    ros::spin();

    return 0;
}
