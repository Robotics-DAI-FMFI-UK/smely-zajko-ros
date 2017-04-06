#include "HokuyoSynthetic.h"

void HokuyoSynthetic::init() {

}

void HokuyoSynthetic::readData() {

}
int r = 1500;
int s = 1;

Int32MultiArray HokuyoSynthetic::getData() {

    r += s;
    if (r < 1500) {
        s = 1;
    }
    if (r > 2000) {
        s = -1;
    }
    Int32MultiArray result;
    for (int i = 0; i < RANGE_DATA_COUNT; i++) {
        result.data.push_back(r);
    }

    return result;

}