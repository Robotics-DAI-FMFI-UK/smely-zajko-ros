#include "ros/ros.h"
#include "robot/Robot.h"
#include "message_types/SbotMsg.h"
#include "message_types/SteeringMsg.h"

using namespace std;

Robot *robot;

void steeringCallback(const message_types::SteeringMsg &msg) {
    robot->set_direction(msg.direction);
    robot->set_speed(msg.speed);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sbot_publisher");
    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<message_types::SbotMsg>("/control/base_data", 3);

    ros::Subscriber subscriber = nh.subscribe("/control/steering", 3, steeringCallback);

    robot = new Robot(publisher);

    ros::Rate loop_rate(20);

    ros::Duration(0.5).sleep();

    while (ros::ok()) {
        robot->publish();

        ros::spinOnce();
    }

    robot->shutdown();

    return 0;
}
