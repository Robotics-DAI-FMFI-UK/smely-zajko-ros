#include "GpsFiles.h"

void GpsFiles::init() {
    file.open("/home/zajko/Desktop/gps.txt", std::ios_base::in);
}

void GpsFiles::readData() {
    if (file.is_open()) {
        std::string line;
        if (getline(file, line)) {
            std::stringstream iss(line);
            float _;
            // for (int i = 0; i < 15; iss >> _, i++);
            iss >> data.longitude;
            iss >> data.latitude;
        } else {
            data.longitude = 0;
            data.latitude = 0;
            file.close();
        }
    } else {
        data.longitude = 0;
        data.latitude = 0;
    }
}

sensor_msgs::NavSatFix GpsFiles::getData() {
    return data;
}
