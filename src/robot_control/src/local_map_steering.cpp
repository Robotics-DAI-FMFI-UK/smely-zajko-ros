#include <stdlib.h>
#include "ros/ros.h"
#include "localMap/LocalMap.h"
#include "message_types/SbotMsg.h"
#include "std_msgs/Int32MultiArray.h"
#include "string.h"

ros::Publisher steeringPublisher;

bool sbot = false, hokuyo = false;

LocalMap* localMap = new LocalMap(600, 600);

void hokuyoCallback(const std_msgs::Int32MultiArray::ConstPtr &array) {
    int i = 0;
    int data[1081];
    for (std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
        data[i] = *it;
        i++;
    }
    localMap->setHokuyoData(data);
}

void sbotCallback(const message_types::SbotMsg &msg) {
    localMap->updateRobotPosition((long) msg.lstep, (long) msg.rstep);
}

void say(const char *msg) {
    char out[256];
    snprintf(out, 255, "echo \"%s\" | espeak -a 200 -p 20 -s 80", msg);
    system(out);
}

void drawMap(const ros::TimerEvent&) {
    cv::imshow("local map", localMap->getGui());
    cv::waitKey(1);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "ros_control");
    ros::NodeHandle nh;

    ros::Subscriber sbot_subscriber = nh.subscribe("/control/base_data", 10, sbotCallback);
    ros::Subscriber hokuyo_subscriber = nh.subscribe("/sensors/hokuyo_publisher", 100, hokuyoCallback);

    ros::Timer update = nh.createTimer(ros::Duration(1), drawMap);

//    cv::imshow("local map", localMap->getGui());
//    cv::waitKey(0);

    ros::spin();

    return 0;
}
