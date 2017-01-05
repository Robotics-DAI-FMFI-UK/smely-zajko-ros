#include "Gps.h"

#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

// TODO: add code from validate
void Gps::init() {

    struct termios oldtio, newtio;

    gps = open("/dev/galileo", O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (gps < 0) {
        // printf("imu not found at %s\n", devName);
        return;
    }

    tcgetattr(gps, &oldtio); /* save current port settings */

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = B4800 | CS8 | CLOCAL | CREAD; // B57600
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME] = 0; /* inter-character timer unused */
    newtio.c_cc[VMIN] = 1;  /* blocking read until 1 char received */

    tcflush(gps, TCIFLUSH);
    tcsetattr(gps, TCSANOW, &newtio);

    do {
        if (!read(gps, b, 1))
            break;
    } while (b[0] != '\n');
}

void Gps::readData() {

    int nread;
    if (!(nread = read(gps, b + bufp, 1))) {

        return;
    }
    if (nread < 0) {
        usleep(3000);

        return;
    }
    if (b[bufp] == '\n') {
        b[bufp] = '\0';
        bufp = 0;
        strncpy(b2, b, 1023);
        b2[1023] = '\0';

        //parseGpsLine(b);

    } else if (bufp < 1023) {
        bufp++;
    } else {
        bufp = 0;
    }
}

sensor_msgs::NavSatFix Gps::getData() {

    return sensor_msgs::NavSatFix();
}
