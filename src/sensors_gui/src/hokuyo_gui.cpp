#include <vector>
#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"

IplImage *result;
int guiWidth = 320;
int guiHeight = 240;

void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr &array) {
    int arr[1081];
    int i = 0;
    for (std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
        arr[i] = *it;
        i++;
    }

    cvSet(result, CV_RGB(255, 255, 255));
    int x, y;
    double brkAngle = -135 / 180.0 * M_PI;
    double deltaAngle = 0.25 / 180 * M_PI;
    for (int i = 0; i < 1081; i++) {
        x = (int) (-arr[i] * sin(brkAngle + i * deltaAngle) / 35 +
                   guiWidth / 2);
        y = (int) (arr[i] * cos(brkAngle + i * deltaAngle) / 35 +
                   guiHeight / 2);
        cvLine(result, cvPoint(guiWidth / 2, guiHeight / 2),
               cvPoint(x, guiHeight - y), cvScalar(0.4, 0.4, 0.4));
    }
    long min = 200000;
    for (int i = 0; i < 1081; i++) {
        if (arr[i] < min && arr[i] > 5000) {
            min = arr[i];
        }
        x = (int) (-arr[i] * sin(brkAngle + i * deltaAngle) / 35 +
                   guiWidth / 2);
        y = (int) (arr[i] * cos(brkAngle + i * deltaAngle) / 35 +
                   guiHeight / 2);
        cvCircle(result, cvPoint(x, guiHeight - y), 2, cvScalar(0.6, 0.8, 0),
                 -1);
    }

    cvShowImage("laser", result);
    cv::waitKey(30);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "hokuyo_gui");
    ros::NodeHandle n;

    result = cvCreateImage(cvSize(320, 240), 32, 3);

    cv::Mat empty_frame = cv::Mat::zeros(320, 240, CV_8UC3);
    cv::namedWindow("laser", CV_WINDOW_AUTOSIZE);
    cv::imshow("laser", empty_frame);
    cv::waitKey(30);

    ros::Subscriber subscriber = n.subscribe("hokuyo_publisher", 100, arrayCallback);

    ros::spin();

    cvDestroyWindow("laser");

    return 0;
}