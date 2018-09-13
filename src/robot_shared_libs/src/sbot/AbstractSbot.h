#ifndef PROJECT_ABSTRACTSBOT_H
#define PROJECT_ABSTRACTSBOT_H


class AbstractSbot {
public:

    virtual int init() = 0;

    virtual message_types::SbotMsg getData() = 0;

    virtual void shutdown() = 0;
};


#endif //PROJECT_ABSTRACTSBOT_H
