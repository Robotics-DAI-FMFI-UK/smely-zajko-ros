#ifndef PROJECT_ABSTRACTCAMERA_H
#define PROJECT_ABSTRACTCAMERA_H

#include <opencv2/opencv.hpp>

class AbstractCamera {

public:

    virtual void init() = 0;

    virtual void readData() = 0;

    virtual cv::Mat getData() = 0;
};


#endif //PROJECT_ABSTRACTCAMERA_H
