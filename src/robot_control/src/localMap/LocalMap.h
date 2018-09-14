#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cmath>
#include <cstdio>
#include <utility>
#include <std_msgs/Float64.h>


class RobotPos {
public:
    double x, y, angle;
};

const double pi = 3.141592653589793238463;

class LocalMap {
public:
    LocalMap(int guiWidth, int guiHeight, ros::Publisher publisher);
    
    // calculate new position from odometry
    void updateRobotPosition(long L, long R);

    void setHokuyoData(int rays[1081]);

    void setGlobalMapData(double currHeading, double nextHeading, double distance);

    void setCompassHeading(double heading);

    void setImageData(unsigned char data[3600]);

    void doUpdate();

    double getHeading();

    cv::Mat getGui();

    RobotPos* getPos();

private:
    // best heading
    double bestHeading;

    // sensor data
    int hokuyo[1081];
    double compassHeading;
    double compassHeading_;
    double currWayHeading;
    double nextWayHeading;
    double wayEndDistance;
    unsigned char cameraData[60][60];

    //sensor data checks;
    bool validHokuyo = false;
    bool validImage = false;

    // gui dimensions
    int guiWidth;
    int guiHeight;

    // matrix to store all data
    double** matrix;
    
    // robot position
    double posX;
    double posY;
    double angle;
    
    // odometry
    long prevTicksL;
    long prevTicksR;

    // last called data (not used if too small)
    long lastTicksL;
    long lastTicksR;

    // util
    int clamp(int val, int max);

    double rescale(double val, double oldMax, double newMax);

    int clampGridX(int x);
    int clampGridY(int y);

    int clampGuiX(int x);
    int clampGuiY(int y);

    int map2gridX(double x);
    int map2gridY(double y);

    int map2guiX(double x);
    int map2guiY(double y);

    ros::Publisher publisher;

    cv::Scalar getMatrixColor(int x, int y);

    // internal methods
    void applyHokuyoData();

    void updateRobotPosition_(long L, long R, bool force);

    void applyCompassHeading();

    void findBestHeading();

    void applyImage();
};
