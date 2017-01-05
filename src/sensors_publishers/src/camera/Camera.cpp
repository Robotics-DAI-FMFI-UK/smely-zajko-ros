#include "Camera.h"

void Camera::init() {
    cap = cv::VideoCapture(0);
    if (!cap.isOpened()) {
        ROS_ERROR("Camera is not initialized");
    }
}

void Camera::readData() {
    cv::Mat frame;
    cap >> frame;
    cv::cvtColor(frame, image, CV_BGR2BGRA);
    // detect edges in image, maybe usable...
    /*cv::GaussianBlur(image, image, cv::Size(7, 7), 1.5, 1.5);
    cv::Canny(image, image, 0, 30, 3);*/
    if (!frame.empty()) {
        cv::flip(image, image, 1);
    }
}

cv::Mat Camera::getData() {

    return image;
}