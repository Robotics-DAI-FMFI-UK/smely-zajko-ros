#ifndef PROJECT_CAMERA_H
#define PROJECT_CAMERA_H

#include "AbstractCamera.h"

#include"ros/ros.h"

class Camera : public AbstractCamera {
    cv::Mat image;
    cv::VideoCapture cap;
public:
    void init();

    void readData();

    cv::Mat getData();
};


#endif //PROJECT_CAMERA_H
