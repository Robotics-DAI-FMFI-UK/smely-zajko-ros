#ifndef PROJECT_ROBOT_H
#define PROJECT_ROBOT_H


#include "AbstractRobot.h"

#include <stdio.h>
#include <string.h>
#include <string>
#include <unistd.h>

class Robot : public AbstractRobot {
public:
    void send_command(const char *command);

    void set_direction(int d);

    void set_max_speed(int maxSpeed);

    void stop_now();

    void set_speed(int s);

    void unblock();

    void ignore_obstacle(bool val);
};


#endif //PROJECT_ROBOT_H
