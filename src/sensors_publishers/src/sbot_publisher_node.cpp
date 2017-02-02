#include "ros/ros.h"
#include "sbot/Sbot.h"
#include "message_types/SbotMsg.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "sbot_publisher");
    ros::NodeHandle nh;
    ros::Publisher sbot_publisher = nh.advertise<message_types::SbotMsg>("sbot_publisher", 10);

    ros::Rate loop_rate(10);

    AbstractSbot *sbot = new Sbot();
    sbot->init();

    ros::Duration(0.5).sleep();

    while (ros::ok()) {
        if (sbot_publisher.getNumSubscribers() > 0) {
            sbot->readData();

            sbot_publisher.publish(sbot->getData());

            ros::spinOnce();

            loop_rate.sleep();
        }
    }

    return 0;
}