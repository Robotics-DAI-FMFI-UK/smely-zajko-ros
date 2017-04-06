#ifndef PROJECT_PREVIOUSALGO_H
#define PROJECT_PREVIOUSALGO_H

#include <cmath>
#include <cstdlib>
#include <vector>
#include "std_msgs/Float64MultiArray.h"

class PreviousAlgo {
    const int DIR_COUNT = 10;
    const int  LASER_VALS_START = 420;
    const int  LASER_VALS_END = 660;

    const int MIN_OBSTACLE_DISTANCE_MM = 300;
    const int MAX_OBSTACLE_DISTANCE_MM = 1600;

    double PROB_GO = 0.9;
    double PROB_NO_GO = 0.1;

    int laser_vals_per_direction;

    double max_range[1081];
public:
    void init();

    double getPath(int arr[1081], int direction);

    std_msgs::Float64MultiArray getPaths(int arr[1081]);
};


#endif //PROJECT_PREVIOUSALGO_H
