#ifndef PROJECT_LOCALIZATIONANDPLANNING_H
#define PROJECT_LOCALIZATIONANDPLANNING_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cfloat>
#include <map>
#include <opencv2/core/types_c.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include "message_types/GpsAngles.h"
#include "../rapidxml/rapidxml.hpp"
#include "../rapidxml/rapidxml_utils.hpp"

extern const int START;
extern const int HEADING_LOADING;
extern const int LOADING;
extern const int HEADING_UNLOADING;
extern const int UNLOADING;
extern const int HEADING_DEST;
extern const int END;

extern int headingState;


using namespace std;
using namespace rapidxml;

class IdDist {
public:
    double id;
    double dist;
};

class FindOnWay {
public:
    sensor_msgs::NavSatFix pointFound;
    double pointId1;
    double pointId2;
    unsigned long pathId;
    int pathPosition;
};

class WayPoint : public sensor_msgs::NavSatFix {
public:
    vector<double> nextPoints;

    // for searching
    double dist;
    double previous;
    int barrier;

    WayPoint() {
        dist = DBL_MAX;
        previous = -1;
        barrier = 0;
    };

    WayPoint(sensor_msgs::NavSatFix src) {
        dist = DBL_MAX;
        previous = -1;
        latitude = src.latitude;
        longitude = src.longitude;
        barrier = 0;
    };
};

class Path {
public:
    vector<double> points;
};

class BoundsLl {
public:
    double minlat;
    double minlon;
    double maxlat;
    double maxlon;
};

class LocalizationAndPlanning {
public:
    vector<Path> paths;
    map<double, WayPoint> points;
    BoundsLl bounds;

    int guiMapWidth;
    int guiMapHeight;
    int guiDebugWidth;
    int guiDebugHeight;

    // main points
    sensor_msgs::NavSatFix destinationPoint;
    sensor_msgs::NavSatFix curPoint;
    sensor_msgs::NavSatFix headingPoint;

    // last known gps position
    sensor_msgs::NavSatFix lastPosition;

    LocalizationAndPlanning(int guiWidth, int guiHeight);

    virtual ~LocalizationAndPlanning();

    // read osm map from xml
    void readMap(char *filename);

    // distance between 2 points on a sphere
    double distance(sensor_msgs::NavSatFix p1, sensor_msgs::NavSatFix p2);

    // gets image of map and main points
    IplImage *getGui();

    // destination set
    void setDestination(sensor_msgs::NavSatFix point);

    void readDestination(char *filename);

    // update state with new gps data
    message_types::GpsAngles update(sensor_msgs::NavSatFix gps);

    // calculate shortest path between two paths
    void calcPath(double strtPoint, double strtPointB, double destPoint,
                  double destPointB);

    // bool distCompare(IdDist i, IdDist j);
    sensor_msgs::NavSatFix reverse(geometry_msgs::Point location);

    // shortest path to destination
    vector<double> bestWay;

private:
    // ellipse parameters
    double ell_a;
    double ell_b;

    // sphere radius
    double EarthRadius;

    // km radius of ellipse for heading point calculation
    double heading_search_radius;

    // fonts for gui
    CvFont font;
    CvFont fontBig;

    // force bestway recalculation when setting new destination
    bool forceRecalc;

    // calc point to linesegment distance and closest point on segment to target
    // point
    pair<double, sensor_msgs::NavSatFix>
    dist_point_linesegment(sensor_msgs::NavSatFix point, sensor_msgs::NavSatFix start, sensor_msgs::NavSatFix end);

    // calc intersections of a line and ellipse
    pair<sensor_msgs::NavSatFix, sensor_msgs::NavSatFix>
    ellipseLineIntersection(sensor_msgs::NavSatFix p0, sensor_msgs::NavSatFix p1);

    // najde bod segmentu a segmente najblizsie k bodu point
    FindOnWay find_on_way(sensor_msgs::NavSatFix point);

    // lon,lat to map x,y
    geometry_msgs::Point convert(sensor_msgs::NavSatFix point);

    // calculates bearing (initial)
    double calc_bearing(sensor_msgs::NavSatFix a, sensor_msgs::NavSatFix b);

    // calculates intersection when going from p1 with initial bearing b1 and
    // path from p2 along bearing b2
    sensor_msgs::NavSatFix
    intersection_of_bearings(sensor_msgs::NavSatFix p1, double b1, sensor_msgs::NavSatFix p2, double b2);

    // calc ellipse parameters
    void calcEllipse(sensor_msgs::NavSatFix point, double km);

    // calc heading point
    sensor_msgs::NavSatFix calcHeadingPoint();
};


#endif //PROJECT_LOCALIZATIONANDPLANNING_H
