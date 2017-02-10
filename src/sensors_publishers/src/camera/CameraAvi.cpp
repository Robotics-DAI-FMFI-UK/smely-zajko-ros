#include "CameraAvi.h"

void CameraAvi::init() {
    cap = cv::VideoCapture(filename);
    if (!cap.isOpened()) {
        ROS_ERROR("error with file");
    }
}

void CameraAvi::readData() {
    cv::Mat frame;
    cap >> frame;
    cv::cvtColor(frame, image, CV_BGR2BGRA);
    if (!frame.empty()) {
        cv::flip(image, image, 0);
        cv::flip(image, image, 1);
    }
}

cv::Mat CameraAvi::getData() {

    return image;
}
