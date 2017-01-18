#include "HokuyoFile.h"

void HokuyoFile::init() {

}

void HokuyoFile::readData() {

}

Int32MultiArray HokuyoFile::getData() {

    Int32MultiArray result;
    int r = rand() % 5;
    for (int i = 0; i < RANGE_DATA_COUNT; i++) {
        result.data.push_back(i * r);
    }

    return result;

}