#include "HokuyoFiles.h"

void HokuyoFiles::init() {
    dir = opendir("/home/jozef/Desktop/smely-zajko/runHokuyo");
}

void HokuyoFiles::readData() {
    if (dir != NULL) {
        if ((ent = readdir(dir)) != NULL) {
            if (strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0) {
                char path[200];
                sprintf(path, "/home/jozef/Desktop/smely-zajko/runHokuyo/%s", ent->d_name);
                std::fstream file(path, std::ios_base::in);
                int a;
                data.data.clear();
                while (file >> a) {
                    data.data.push_back(a);
                }
                file.close();
            }
        } else {
            data = Int32MultiArray();
            closedir(dir);
        }
    }
}

Int32MultiArray HokuyoFiles::getData() {

    return data;
}