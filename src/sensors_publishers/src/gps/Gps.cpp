#include "Gps.h"

double stringToDouble(string s) {
    double returnValue = 0;
    std::istringstream istr(s);

    istr >> returnValue;

    return (returnValue);
}

double nmeaToDecimal(string position) {
    double pos = stringToDouble(position);
    double deg = floor(pos / 100);
    double dec_pos = deg + ((pos - (deg * 100)) / 60);

    return dec_pos;
}

vector<string> splitStringByComma(string input) {

    vector<string> returnVector;
    stringstream ss(input);
    string element;

    while (getline(ss, element, ',')) {
        returnVector.push_back(element);
    }

    return returnVector;
}

bool isValidGPGGA(const vector<string> elementVector) {
    if ((elementVector[0] != "$GPGGA")
       && (elementVector[0] != "$GNGGA")) {
        return false;
    }

    if (elementVector.size() != 15) {
        return false;
    }
    if (atoi(elementVector[6].c_str()) == 0) {
        return false;
    }
    if (elementVector[4].length() < 9) {
        return false;
    }
    if (elementVector[2].length() < 9) {
        return false;
    }

    return atoi(elementVector[7].c_str()) != 0;
}

void Gps::init() {

    struct termios oldtio, newtio;

    gps = open(dev_name, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (gps < 0) {
        ROS_ERROR("GPS not found at %s", dev_name);

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

    while (true) {
        ssize_t nread;
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
            break;
        } else if (bufp < 1023) {
            bufp++;
        } else {
            bufp = 0;
        }
    }
}

sensor_msgs::NavSatFix Gps::getData() {
    // ROS_ERROR(b);

    return parseLine(b);
}

sensor_msgs::NavSatFix Gps::parseLine(const char *s) {

    sensor_msgs::NavSatFix fix;

    vector<string> splittedLine = splitStringByComma(s);

    if (isValidGPGGA(splittedLine)) {
        for (int i = 0; i < splittedLine.size(); i++) {
            cout << splittedLine[i] << " ";
        }

        cout << '\n';

        fix.latitude = nmeaToDecimal(splittedLine[2]);
        fix.longitude = nmeaToDecimal(splittedLine[4]);
        if (splittedLine[3] == "S") {
            fix.latitude = -fix.latitude;
        }
        if (splittedLine[5] == "W") {
            fix.longitude = -fix.longitude;
        }
        fix.altitude = stringToDouble(splittedLine[9]);
        //numberSatellites = atoi(elementVector[7].c_str());
    }


    return fix;
}
