#include "ros/ros.h"
#include <opencv2/opencv.hpp>

#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>
#include "std_msgs/Int32MultiArray.h"

#define guiMapWidth 500
#define guiDebugWidth 500
#define guiDebugHeight 50
#define guiMapHeight 450


CvFont font;
CvFont fontBig;
/*
void test() {
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.3, 0.3);
    cvInitFont(&fontBig, CV_FONT_HERSHEY_SIMPLEX, 0.6, 0.6);
    IplImage *result = cvCreateImage(
            cvSize(guiMapWidth, guiMapHeight + guiDebugHeight), 32, 3);
    // biele pozadie
    cvSet(result, CV_RGB(255, 255, 255));
    // cesty
    for (unsigned long i = 0; i < paths.size(); i++) {
        if (paths.at(i).points.size() > 1) {
            geometry_msgs::Point p1 = convert(points.at(paths.at(i).points[0]));

            for (int j = 1; j < paths.at(i).points.size(); j++) {
                geometry_msgs::Point p2 = convert(points.at(paths.at(i).points[j]));

                cvLine(result, cvPoint(p1.x, p1.y), cvPoint(p2.x, p2.y),
                       cvScalar(0, 0, 0));
                p1 = p2;
            }
        }
    }

    // best way
    if (bestWay.size() > 0) {
        double bluuu = (double) 1 / bestWay.size(); // gradient
        geometry_msgs::Point p1 = convert(points[bestWay[0]]);
        for (int i = 1; i < bestWay.size(); i++) {
            geometry_msgs::Point p2 = convert(points[bestWay[i]]);
            cvLine(result, cvPoint(p1.x, p1.y), cvPoint(p2.x, p2.y),
                   cvScalar(0.5 + 0.5 * (i * (bluuu)), 0, 0), 2);
            p1 = p2;
        }
    }

    // raw gps - kruh
    geometry_msgs::Point o2 = convert(lastPosition);
    cvCircle(result, cvPoint(o2.x, o2.y), 3, cvScalar(0, 1, 0), -1);

    // ciel - stvorcek
    geometry_msgs::Point o3 = convert(destinationPoint);
    cvRectangle(result, cvPoint(o3.x - 5, o3.y - 5),
                cvPoint(o3.x + 5, o3.y + 5), cvScalar(0.8, 0.8, 0.8), -1);

    // poloha - krizik
    geometry_msgs::Point o4 = convert(curPoint);
    cvLine(result, cvPoint(o4.x - 4, o4.y - 4), cvPoint(o4.x + 4, o4.y + 4),
           cvScalar(0, 0, 1), 2);
    cvLine(result, cvPoint(o4.x - 4, o4.y + 4), cvPoint(o4.x + 4, o4.y - 4),
           cvScalar(0, 0, 1), 2);

    // medziciel - plny kruh
    geometry_msgs::Point o1 = convert(headingPoint);
    cvCircle(result, cvPoint(o1.x, o1.y), 3, cvScalar(1, 0, 1), -1);

    // elipsa okolia
    sensor_msgs::NavSatFix dummy;
    dummy.longitude = curPoint.longitude;
    dummy.latitude = curPoint.latitude + ell_a;
    geometry_msgs::Point o8 = convert(dummy);
    dummy.longitude = curPoint.longitude + ell_b;
    dummy.latitude = curPoint.latitude;
    geometry_msgs::Point o9 = convert(dummy);
    cvEllipse(result, cvPoint(o4.x, o4.y),
              cvSize(abs(o4.x - o9.x), abs(o4.y - o8.y)), 0, 0, 360,
              cvScalar(0.5, 0.5, 0.5));

    // draw debug
    if (guiDebugHeight > 0) {
        // base
        cvRectangle(result, cvPoint(0, guiMapHeight),
                    cvPoint(guiDebugWidth, guiMapHeight + guiDebugHeight),
                    cvScalar(1, 1, 1), -1);
        cvLine(result, cvPoint(0, guiMapHeight),
               cvPoint(guiDebugWidth, guiMapHeight), cvScalar(0, 0, 0), 1);

        // distances
        cvPutText(result, "gps", cvPoint(5, guiMapHeight + 15), &font,
                  cvScalar(0, 0, 0));
        cvCircle(result, cvPoint(30, guiMapHeight + 15), 3, cvScalar(0, 1, 0),
                 -1);
        cvPutText(result, "->curPos", cvPoint(35, guiMapHeight + 15), &font,
                  cvScalar(0, 0, 0));
        cvLine(result, cvPoint(90 - 4, guiMapHeight + 15 - 4),
               cvPoint(90 + 4, guiMapHeight + 15 + 4), cvScalar(0, 0, 1), 2);
        cvLine(result, cvPoint(90 - 4, guiMapHeight + 15 + 4),
               cvPoint(90 + 4, guiMapHeight + 15 - 4), cvScalar(0, 0, 1), 2);

        std::stringstream diststr;
        diststr << distance(lastPosition, curPoint) * 1000 << "m";
        cvPutText(result, diststr.str().c_str(), cvPoint(5, guiMapHeight + 40),
                  &fontBig, cvScalar(0, 0, 0));

        cvPutText(result, "curPos", cvPoint(120, guiMapHeight + 15), &font,
                  cvScalar(0, 0, 0));
        cvLine(result, cvPoint(160 - 4, guiMapHeight + 15 - 4),
               cvPoint(160 + 4, guiMapHeight + 15 + 4), cvScalar(0, 0, 1), 2);
        cvLine(result, cvPoint(160 - 4, guiMapHeight + 15 + 4),
               cvPoint(160 + 4, guiMapHeight + 15 - 4), cvScalar(0, 0, 1), 2);
        cvPutText(result, "->dest", cvPoint(165, guiMapHeight + 15), &font,
                  cvScalar(0, 0, 0));
        cvRectangle(result, cvPoint(207 - 5, guiMapHeight + 15 - 5),
                    cvPoint(207 + 5, guiMapHeight + 15 + 5),
                    cvScalar(0.8, 0.8, 0.8), -1);

        diststr.str("");
        diststr << distance(destinationPoint, curPoint) * 1000 << "m";
        cvPutText(result, diststr.str().c_str(),
                  cvPoint(120, guiMapHeight + 40), &fontBig, cvScalar(0, 0, 0));
        // bestway info
        diststr.str("seg");
        cvPutText(result, diststr.str().c_str(),
                  cvPoint(340, guiMapHeight + 20), &font, cvScalar(0, 0, 0));
        diststr.str("");
        diststr << bestWay.size() - 1;
        cvPutText(result, diststr.str().c_str(),
                  cvPoint(335, guiMapHeight + 40), &fontBig, cvScalar(0, 0, 0));
    }
}*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "localization_and_planning_gui");
    ros::NodeHandle n;

    return 0;
}