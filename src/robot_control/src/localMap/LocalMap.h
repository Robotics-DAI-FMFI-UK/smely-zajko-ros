#ifndef _LOCALMAP_H_
#define _LOCALMAP_H_

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cmath>
#include <cstdio>
#include <utility>
#include <std_msgs/Float64.h>
#include <pthread.h>
#include "Graph.h"

class Planner;

#define EVALUATED_IMAGE_PACKET_TYPE   1
#define POSITION_PACKET_TYPE 3
#define DEPTH_MAP_PACKET_TYPE 4
#define EVALUATED_IMAGE_LISTENER_PORT 9771
#define POSITION_AND_DEPTH_MAP_PORT 9772

#define LOCALMAP_LOG_IMAGE_PATH "/home/zajko/logs/"

void log_msg(const char *msg);
void log_msg(const char *msg, double val);
void log_msg(const char *msg, double val1, double val2);
double angleDiffAbs(double a, double b);

extern const int gridSize;
extern const int gridWidth;
extern const int gridHeight;
extern const int mapWidth;
extern const int mapHeight;

using namespace std;


class RobotPos {
public:
    double x, y, angle;
};

class LocalMap {

friend class Planner;

public:
    LocalMap(int guiWidth, int guiHeight, ros::Publisher publisher);
    
    // calculate new position from odometry
    void updateRobotPosition(long L, long R);

    void setPose(double x, double y, double a);

    void setHokuyoData(int rays[1081]);

    void setRpLidarData(int rays, double* distances, double* angles);

    void setGlobalMapData(double currHeading, double nextHeading, double distance);

    void setCompassHeading(double heading);

    void setImageData(unsigned char data[3600]);

    void setDepthMap(unsigned char *data);

    void doUpdate();

    double getHeading();

    cv::Mat getGui();
    
    void addArrows(cv::Mat &result);

    RobotPos* getPos();
        
private:
    // best heading
    double scores[360];
    volatile double bestHeading;
    
    //toggle between Fikar algorithm and Slimak algorithm of local map
    int use_slimak_heading = 1;
    int use_random_intersection_lines = 0;
    
    volatile double bestSlimakHeading;

    // sensor data
    int hokuyo[1081];
    double compassHeading;
    double compassHeading_;
    double currWayHeading;
    double nextWayHeading;
    double wayEndDistance;
    unsigned char cameraData[60][60];
    unsigned char depthMap[60][60];
    int rpRays;
    double rpDistances[400];
    double rpAngles[400];


	double** mask_val;
	int** mask_count;

	double** depth_mask_val;
	int** depth_mask_count;
	
    //sensor data checks;
    volatile bool validHokuyo = false;
    volatile bool validRpLidar = false;
    volatile bool validImage = false;
    volatile bool validDepthMap = false;

    // gui dimensions
    int guiWidth;
    int guiHeight;

    // matrix to store all data
    double** matrix;
    double** matrix_cam;
    
    // robot position
    volatile double posX;
    volatile double posY;
    volatile double angle;
    
    // odometry
    long prevTicksL;
    long prevTicksR;

    // last called data (not used if too small)
    long lastTicksL;
    long lastTicksR;
    
    // to pass from getGui() to addArrows()
    int guiShiftX, guiShiftY;
    
    // last trajectory that has been found
    vector<pair<int, int>> slimak_trajectory;
    
    pthread_mutex_t trajectory_lock;

    Graph visualizedGraph;
    Planner *planner;

    // util
    int clamp(int val, int max);

    double rescale(double val, double oldMax, double newMax);

    int clampGridX(int x);
    int clampGridY(int y);

    int clampGuiX(int x);
    int clampGuiY(int y);

    int map2gridX(double x);
    int map2gridY(double y);

    int map2gridX_(double x);
    int map2gridY_(double y);

    int map2guiX(double x);
    int map2guiY(double y);

    int map2guiXFULL(double x);
    int map2guiYFULL(double y);

    ros::Publisher publisher;

    cv::Scalar getMatrixColor(int x, int y);

    // internal methods
    void eraseAustralia();

    void decayMap();

    void applyRay_clear_and_mark(double sensorX, double sensorY, double rayAngle, double rayLen);
    
    void applyHokuyoData();

    void applyRpLidarData();

    void updateRobotPosition_(long L, long R, bool force);

    void applyCompassHeading();

    void findBestHeading();
    
    void findBestHeading_slimak();

    void applyImage();
    
    void applyZed();

    void applyDepthMap();
    
    void find_border_point_for_angle(double wished_heading, double goal_position[], double map_width);
};

#endif
