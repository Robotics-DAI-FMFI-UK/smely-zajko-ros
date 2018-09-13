#ifndef PROJECT_ROBOT_H
#define PROJECT_ROBOT_H


#include "ros/ros.h"
#include "../../../robot_shared_libs/src/sbot/Sbot.h"
#include <stdio.h>
#include <string.h>
#include <string>
#include <unistd.h>
#include "message_types/SbotMsg.h"


class Robot {
    Sbot *sbot;
    ros::Publisher publisher;
public:
    Robot(ros::Publisher publisher);

    void send_command(const char *command);

    void set_direction(int d);

    void set_max_speed(int maxSpeed);

    void stop_now();

    void set_speed(int s);

    void unblock();

    void ignore_obstacle(bool val);

    void publish();
  
    void get_latest_data(message_types::SbotMsg *msg);

    void shutdown();

private:
    bool isInit;
};


#endif //PROJECT_ROBOT_H
