#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float64MultiArray.h>

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"

IplImage *result;
int hokuyo_results[1081];
double hokuyo_weights[11];

int guiWidth = 320;
int guiHeight = 240;

void hokuyoCallback(const std_msgs::Int32MultiArray::ConstPtr &array) {
    int i = 0;
    for (std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
        hokuyo_results[i] = *it;
        i++;
    }
}

void hokuyoAlgoCallback(const std_msgs::Float64MultiArray::ConstPtr &array) {
    int i = 0;
    for (std::vector<double>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
        hokuyo_weights[i] = *it;
        i++;
    }
}

void render_window() {
    cvSet(result, CV_RGB(255, 255, 255));
    int x, y;
    double brkAngle = -135 / 180.0 * M_PI;
    double deltaAngle = 0.25 / 180 * M_PI;

    std::set<int> directions;
    directions.insert(705);
    directions.insert(672);
    directions.insert(639);
    directions.insert(606);
    directions.insert(573);
    directions.insert(540);
    directions.insert(507);
    directions.insert(474);
    directions.insert(441);
    directions.insert(408);
    directions.insert(375);

    for (int i = 0; i < 1081; i++) {
        x = (int) (-hokuyo_results[i] * sin(brkAngle + i * deltaAngle) / 35 +
                   guiWidth / 2);
        y = (int) (hokuyo_results[i] * cos(brkAngle + i * deltaAngle) / 35 +
                   guiHeight / 2);
        cvLine(result, cvPoint(guiWidth / 2, guiHeight / 2),
               cvPoint(x, guiHeight - y), cvScalar(0.4, 0.4, 0.4));
        cvCircle(result, cvPoint(x, guiHeight - y), 2, cvScalar(0.6, 0.8, 0), -1);
    }

    for (std::set<int>::iterator it = directions.begin(); it != directions.end(); ++it) {
        x = (int) (-hokuyo_results[*it] * sin(brkAngle + *it * deltaAngle) / 35 +
                   guiWidth / 2);
        y = (int) (hokuyo_results[*it] * cos(brkAngle + *it * deltaAngle) / 35 +
                   guiHeight / 2);
        cvCircle(result, cvPoint(x, guiHeight - y), 2, cvScalar(0.0, 1, 0.0), -1);
    }

    double max = -INFINITY;
    int max_index = 0;

    for (int i = 0; i < 11; i++) {
        cvCircle(result, cvPoint(guiWidth / 11 * (i + 0.5), guiHeight - guiHeight * hokuyo_weights[i]), 2,
                 cvScalar(0.6, 0.8, 0),
                 -1);
        if (hokuyo_weights[i] > max) {
            max = hokuyo_weights[i];
            max_index = i;
        }
    }

    cvCircle(result, cvPoint(guiWidth / 11 * (max_index + 0.5), guiHeight - guiHeight * max), 3,
             cvScalar(0, 0, 1),
             -1);


    cvShowImage("laser", result);
    cv::waitKey(30);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "hokuyo_gui");
    ros::NodeHandle n;

    result = cvCreateImage(cvSize(guiWidth, guiHeight), 32, 3);

    cv::Mat empty_frame = cv::Mat::zeros(guiWidth, guiHeight, CV_8UC3);
    cv::namedWindow("laser", CV_WINDOW_AUTOSIZE);
    cv::imshow("laser", empty_frame);
    cv::waitKey(30);

    ros::Subscriber subscriber = n.subscribe("/sensors/hokuyo_publisher", 100, hokuyoCallback);
    ros::Subscriber hokuyo_subscriber = n.subscribe("/control/hokuyo_algo", 100, hokuyoAlgoCallback);

    ros::Rate loop_rate(30);

    while (ros::ok()) {
        render_window();

        ros::spinOnce();

        loop_rate.sleep();
    }

    cvDestroyWindow("laser");

    return 0;
}