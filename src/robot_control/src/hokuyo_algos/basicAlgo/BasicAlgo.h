#ifndef PROJECT_BASICALGO_H
#define PROJECT_BASICALGO_H

#include <cmath>
#include <cstdlib>
#include <vector>
#include "std_msgs/Float64MultiArray.h"

class BasicAlgo {
    const int DIR_COUNT = 10;
    const double PARABOLA_RANGE_FORWARD = 2500.0;
    const double PARABOLA_RANGE_SIDE = 800.0;

    std::vector<int> directions;
    double max_range[1081];
public:
    void init();

    double getPath(int arr[1081], int direction);

    std_msgs::Float64MultiArray getPaths(int arr[1081]);
};


#endif //PROJECT_BASICALGO_H
