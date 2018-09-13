#include <stdio.h>
#include "Robot.h"

using namespace std;

Robot::Robot(ros::Publisher publisher) {
    this->publisher = publisher;
    sbot = new Sbot();
    isInit = (sbot->init() == 0);
}

void Robot::send_command(const char *command) {
    if (!isInit) return;
    write(sbot->fdR[1], command, strlen(command));
}

void Robot::set_direction(int d) {
    char buff[32];

    sprintf(buff, "d %d;", d);
    send_command(buff);
}

void Robot::set_max_speed(int maxSpeed) {
    char buff[32];

    sprintf(buff, "m %d;", maxSpeed);
    send_command(buff);
}

void Robot::stop_now() {
    char buff[32];
    sprintf(buff, "/");
    send_command(buff);
}

void Robot::set_speed(int s) {
    char buff[32];

    sprintf(buff, "s %d;", s * 2);
    send_command(buff);
}

void Robot::unblock() {
    send_command("u;");
}

void Robot::ignore_obstacle(bool val) {
    send_command(val ? "i;" : "o;");
}

void Robot::publish() 
{
   message_types::SbotMsg msg;
   sbot->getData(&msg); 
   publisher.publish(msg);
}

void Robot::get_latest_data(message_types::SbotMsg *msg)
{
   sbot->getData(msg);
}

void Robot::shutdown() {
    sbot->shutdown();
}

