#include"ros/ros.h"
#include "std_msgs/Int32MultiArray.h"

#include "hokuyo/Hokuyo.h"

using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "hokuyo_publisher");
    ros::NodeHandle n;
    ros::Publisher hokuyo_publisher = n.advertise<std_msgs::Int32MultiArray>("hokuyo_publisher", 100);

    ros::Rate loop_rate(10);

    AbstractHokuyo *hokuyo = new Hokuyo();
    hokuyo->init();

    ros::Duration(0.5).sleep();

    while (ros::ok()) {
        hokuyo->readData();

        hokuyo_publisher.publish(hokuyo->getData());

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}