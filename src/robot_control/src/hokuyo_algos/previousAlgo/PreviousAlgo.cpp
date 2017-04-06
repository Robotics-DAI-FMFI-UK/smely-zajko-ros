#include "PreviousAlgo.h"

void PreviousAlgo::init() {
    this->laser_vals_per_direction = (LASER_VALS_END - LASER_VALS_START) / DIR_COUNT;
}

double PreviousAlgo::getPath(int *arr, int direction) {
    direction = 10 - direction;
    int start = LASER_VALS_START + (direction * laser_vals_per_direction);
    int end = LASER_VALS_START + ((direction + 1) * laser_vals_per_direction);

    int min_distance = arr[start];
    for (int i = start; i < end; ++i) {
        int curr_distance = arr[i];
        if (curr_distance <= MIN_OBSTACLE_DISTANCE_MM) {
            continue;
        }

        if (curr_distance < min_distance) {
            min_distance = curr_distance;
        }
    }

    if (direction < DIR_COUNT / 3) {
        for (int i = 180; i < LASER_VALS_START; ++i) {
            int curr_distance = arr[i];
            if (curr_distance <= MIN_OBSTACLE_DISTANCE_MM) {
                continue;
            }

            if (curr_distance < min_distance) {
                min_distance = curr_distance;
            }
        }
    } else if (direction > 2 * DIR_COUNT / 3) {
        for (int i = LASER_VALS_END; i < 901; ++i) {
            int curr_distance = arr[i];
            if (curr_distance <= MIN_OBSTACLE_DISTANCE_MM) {
                continue;
            }

            if (curr_distance < min_distance) {
                min_distance = curr_distance;
            }
        }
    }

    if (min_distance < MAX_OBSTACLE_DISTANCE_MM) {
        return PROB_NO_GO;
    } else {
        return PROB_GO;
    }
}

std_msgs::Float64MultiArray PreviousAlgo::getPaths(int *arr) {

    std_msgs::Float64MultiArray result;
    for (int i = 0; i <= DIR_COUNT; i++) {
        result.data.push_back(this->getPath(arr, i));
    }

    return result;
}
