#include "Robot.h"

using namespace std;

void Robot::send_command(const char *command) {
    write(1, command, strlen(command));
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
    if (val) {
        send_command("i;");
    } else {
        send_command("o;");
    }
}