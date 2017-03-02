#ifndef PROJECT_ABSTRACTSBOT_H
#define PROJECT_ABSTRACTSBOT_H

#include "message_types/SbotMsg.h"

class AbstractSbot {
public:

    virtual void init() = 0;

    virtual void readData() = 0;

    virtual message_types::SbotMsg getData() = 0;
};


#endif //PROJECT_ABSTRACTSBOT_H
