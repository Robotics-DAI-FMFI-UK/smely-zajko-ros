#include "Imu.h"

void Imu::init() {
    imu = open(dev_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (imu < 0) {
        ROS_ERROR("imu not found");
        return;
    }

    struct termios oldtio, newtio;

    tcgetattr(imu, &oldtio); /* save current port settings */

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = B38400 | CS8 | CLOCAL | CREAD; // B57600
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME] = 0; /* inter-character timer unused */
    newtio.c_cc[VMIN] = 1;  /* blocking read until 1 char received */

    tcflush(imu, TCIFLUSH);
    tcsetattr(imu, TCSANOW, &newtio);
}

void Imu::readData() {
    char b[1024];
    int readptr = 0;
    int nread;
    do {
        if ((nread = read(imu, b + readptr, 1)) < 0) {
            if ((errno == EAGAIN) || (errno == EWOULDBLOCK)) {
                usleep(25000);
                continue;
            } else
                return;
        }
        readptr += nread;
    } while ((b[readptr - 1] != '\n'));
    b[readptr] = 0;

    int xAngle, yAngle, zAngle;
    sscanf(b, "%d %d %d", &xAngle, &yAngle, &zAngle);

    result.orientation.x = xAngle + COMPASS_ALIGNMENT;
    result.orientation.y = yAngle;
    result.orientation.z = zAngle;
}

sensor_msgs::Imu Imu::getData() {
    
    return result;
}