#ifndef PROJECT_ABSTRACTROBOT_H
#define PROJECT_ABSTRACTROBOT_H


#include "message_types/SbotMsg.h"

class AbstractRobot {

public:
    virtual void send_command(const char *command) = 0;

    virtual void set_direction(int d)= 0;

    virtual void set_max_speed(int maxSpeed)= 0;

    virtual void stop_now()= 0;

    virtual void set_speed(int s)= 0;

    virtual void unblock()= 0;

    virtual void ignore_obstacle(bool val)= 0;
   
    virtual void publish() = 0;

    virtual message_types::SbotMsg get_latest_data() = 0;

//    virtual void shutdown() = 0;
};


#endif //PROJECT_ABSTRACTROBOT_H
