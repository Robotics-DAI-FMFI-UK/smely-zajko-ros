#include "HokuyoSynthetic.h"

void HokuyoSynthetic::init() {

}

void HokuyoSynthetic::readData() {

}

Int32MultiArray HokuyoSynthetic::getData() {

    Int32MultiArray result;
    int r = rand() % 5;
    for (int i = 0; i < RANGE_DATA_COUNT; i++) {
        result.data.push_back(i * r);
    }

    return result;

}