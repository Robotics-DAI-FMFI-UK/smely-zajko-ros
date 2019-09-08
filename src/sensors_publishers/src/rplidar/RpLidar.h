#ifndef PROJECT_RPLIDAR_H
#define PROJECT_RPLIDAR_H

#include <rplidar.h>

#include "ros/ros.h"

#define MAX_LIDAR_DATA_COUNT 720

typedef struct lidarstruct {
  uint16_t count;
  uint8_t quality[MAX_LIDAR_DATA_COUNT];
  uint16_t distance[MAX_LIDAR_DATA_COUNT];   // [mm]
  uint16_t angle[MAX_LIDAR_DATA_COUNT];      // [deg * 64]
} lidar_data_type;

class RpLidar 
{
    pthread_mutex_t m_read;

public:

    int program_runs;

    void init();
    void get_data(lidar_data_type *buffer);
    void stop();

    friend void *lidar_thread(void *args);

private:

    pthread_mutex_t lidar_lock;
    rplidar_response_measurement_node_t *lidar_data;
    size_t lidar_data_count;
    rp::standalone::rplidar::RPlidarDriver *drv;
    rplidar_response_measurement_node_t *local_data;
    lidar_data_type lidar_output_data;
    pthread_t t;

  int connect_lidar();
};


#endif //PROJECT_HOKUYO_H
