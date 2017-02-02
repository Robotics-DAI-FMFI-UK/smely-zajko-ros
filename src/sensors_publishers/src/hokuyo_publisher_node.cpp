#include"ros/ros.h"
#include "std_msgs/Int32MultiArray.h"

#include "hokuyo/Hokuyo.h"
#include "hokuyo/HokuyoSynthetic.h"

using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "hokuyo_publisher");
    ros::NodeHandle nh;
    ros::Publisher hokuyo_publisher = nh.advertise<std_msgs::Int32MultiArray>("hokuyo_publisher", 100);

    ros::Rate loop_rate(10);

    AbstractHokuyo *hokuyo = new HokuyoSynthetic();
    hokuyo->init();

    ros::Duration(0.5).sleep();

    while (ros::ok()) {
        if (hokuyo_publisher.getNumSubscribers() > 0) {
            hokuyo->readData();

            hokuyo_publisher.publish(hokuyo->getData());

            ros::spinOnce();

            loop_rate.sleep();
        }
    }

    return 0;
}