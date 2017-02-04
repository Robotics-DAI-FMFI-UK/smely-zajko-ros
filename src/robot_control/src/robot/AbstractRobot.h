#ifndef PROJECT_ABSTRACTROBOT_H
#define PROJECT_ABSTRACTROBOT_H


class AbstractRobot {

public:
    virtual void send_command(const char *command) = 0;
};


#endif //PROJECT_ABSTRACTROBOT_H
