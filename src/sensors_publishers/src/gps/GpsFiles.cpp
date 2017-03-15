#include "GpsFiles.h"

void GpsFiles::init() {
    dir = opendir("/home/jozef/Desktop/smely-zajko/logs");
}

void GpsFiles::readData() {
    if (dir != NULL) {
        if ((ent = readdir(dir)) != NULL) {
            if (strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0) {
                char path[200];
                sprintf(path, "/home/jozef/Desktop/smely-zajko/logs/%s", ent->d_name);
                std::fstream file(path, std::ios_base::in);
                long _;
                for(int i = 0; i < 14; i++, file >> _);
                file >> data.longitude;
                file >> data.latitude;
                file.close();
            }
        } else {
            data = sensor_msgs::NavSatFix();
            closedir(dir);
        }
    }
}

sensor_msgs::NavSatFix GpsFiles::getData() {
    return data;
}
