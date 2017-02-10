#ifndef PROJECT_CAMERAAVI_H
#define PROJECT_CAMERAAVI_H


#include "AbstractCamera.h"

#include"ros/ros.h"

#define filename "/home/jozef/Downloads/example_input.avi"

class CameraAvi : public AbstractCamera {
    cv::Mat image;
    cv::VideoCapture cap;
public:
    void init();

    void readData();

    cv::Mat getData();
};


#endif //PROJECT_CAMERAAVI_H
