#include "BasicAlgo.h"

double ray_to_alpha(int ray) {
    return 90.0 - (ray - 180.0) / 4.0;
}

void BasicAlgo::init() {
    directions.push_back(650);
    directions.push_back(630);
    directions.push_back(610);
    directions.push_back(585);
    directions.push_back(560);
    directions.push_back(525);
    directions.push_back(500);
    directions.push_back(470);
    directions.push_back(445);
    directions.push_back(420);
    directions.push_back(400);
    double par_c = PARABOLA_RANGE_FORWARD;
    double par_a = -PARABOLA_RANGE_FORWARD / (PARABOLA_RANGE_SIDE * PARABOLA_RANGE_SIDE);

    for (int ray = 180; ray <= 900; ray++) {
        double alpha = (ray_to_alpha(ray) + 90.0) / 180.0 * M_PI;
        if (ray == 540) {
            max_range[ray] = PARABOLA_RANGE_FORWARD;
        } else {
            max_range[ray] = -((0.0 - sin(alpha) +
                                sqrt(sin(alpha) * sin(alpha) - 4 * par_a * par_c * cos(alpha) * cos(alpha)))
                               / (2 * par_a * cos(alpha) * cos(alpha)));
        }
    }
}

double BasicAlgo::getPath(int arr[1081], int direction) {

//    int direction_index = 540 + abs(6 - direction) * 20 * ((6 - direction < 0) ? 1 : -1);
    int direction_laser_begin = std::max(180, directions[direction] - 360);

    //  int direction_laser_begin = direction_index - 360;

    int direction_laser_end = std::min(900, directions[direction] + 360);
//    int direction_laser_end = direction_index + 360;

    double sum = 0;
    double sum1 = 0;

    //printf("------------------------ dir=%d\n", direction);

    for (int i = direction_laser_begin; i < direction_laser_end; i++) {

        int relative_ray = i - (directions[direction] - 1081 / 2);
        double normalized_alpha = ray_to_alpha(relative_ray) / 90.0;

        double ray_weight = (1 - fabs(normalized_alpha));
        ray_weight *= ray_weight;
        ray_weight *= ray_weight;
        sum += ray_weight;
        double laser_limited = (arr[i] > max_range[relative_ray]) ? (PARABOLA_RANGE_FORWARD) : (arr[i]);
        sum1 += ray_weight * laser_limited / PARABOLA_RANGE_FORWARD; // max_range[i];

        //printf("%d: %lf  %lf\n", i, (1 - fabs(normalized_alpha)), (1 - fabs(normalized_alpha)) * laser_limited / PARABOLA_RANGE_FORWARD);
    }


    double result = sum1 / sum;

    return result;
}

std_msgs::Float64MultiArray BasicAlgo::getPaths(int arr[1081]) {
    std_msgs::Float64MultiArray result;
    for (int i = 0; i <= DIR_COUNT; i++) {
        result.data.push_back(this->getPath(arr, i));
    }

    return result;
}