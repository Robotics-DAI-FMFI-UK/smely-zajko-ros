#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <unistd.h>

#include "RpLidar.h"

#define LIDAR_PORT "/dev/rplidar"
#define LIDAR_BAUD_RATE 115200

using namespace rp::standalone::rplidar;

int RpLidar::connect_lidar()
{
    // create the driver instance
    drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
    sleep(1);

    if (!drv) 
    {
        printf("rplidar: insufficent memory\n");
        return 0;
    }

    rplidar_response_device_health_t healthinfo;
    rplidar_response_device_info_t devinfo;
    // try to connect
    if (IS_FAIL(drv->connect(LIDAR_PORT, LIDAR_BAUD_RATE))) 
    {
        printf("rplidar: error, cannot bind to the specified serial port\n");
        return 0;
    }

    // retrieving the device info
    ////////////////////////////////////////
    u_result op_result = drv->getDeviceInfo(devinfo);

    if (IS_FAIL(op_result)) 
    {
        if (op_result == RESULT_OPERATION_TIMEOUT) 
            // you can check the detailed failure reason
            printf("rplidar: error, operation time out\n");
        else 
            printf("rplidar: error, unexpected error %d\n", op_result);
            // other unexpected result
        return 0;
    }

    // check the device health
    ////////////////////////////////////////
    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) 
     // the operation has succeeded
        switch (healthinfo.status) 
        {
            case RPLIDAR_STATUS_OK:
                printf("RPLidar health status : OK.\n");
                break;
            case RPLIDAR_STATUS_WARNING:
                printf("RPLidar health status : Warning, lidar errorcode: %d\n", healthinfo.error_code);
                break;
            case RPLIDAR_STATUS_ERROR:
                printf("RPLidar health status : Error, lidar errorcode: %d\n", healthinfo.error_code);
                return 0;
        }
    else 
    {
        printf("Error, cannot retrieve the lidar health code: %d\n", op_result);
        return 0;
    }


    if (healthinfo.status == RPLIDAR_STATUS_ERROR) 
    {
        printf("Error, rplidar internal error detected. Please reboot the device to retry.");
        // enable the following code if you want rplidar to be reboot by software
        // drv->reset();
        return 0;
    }

    drv->startMotor();
    sleep(2);
    printf("lidar connected\n");
    return 1;
}

void RpLidar::get_data(lidar_data_type *buffer)
{
    pthread_mutex_lock(&lidar_lock);
    for (size_t i = 0; i < lidar_data_count; ++i) 
    {
        buffer->quality[i] = lidar_data[i].sync_quality >> 2;   // syncbit:1;syncbit_inverse:1;quality:6;
        buffer->angle[i] = lidar_data[i].angle_q6_checkbit >> 1; // check_bit:1;angle_q6:15;
        buffer->distance[i] = lidar_data[i].distance_q2;
    }
    buffer->count = lidar_data_count;
    pthread_mutex_unlock(&lidar_lock);
}

void *lidar_thread(void *args)
{
    RpLidar *me =  (RpLidar *) args;

    while (me->program_runs)
    {
        int erri = 0;
        while (erri < 30) 
        {
            if (IS_FAIL(me->drv->startScan())) 
            {
                printf("rplidar error, cannot start the scan operation. Trying again.\n");
                ++erri;
                usleep(50000);
            } else break;
        
        }
        if (erri == 30) 
        { 
            printf("rplidar error, cannot start the scan operation. End.\n");
            break;
        }

        u_result ans;    
        size_t local_data_count = MAX_LIDAR_DATA_COUNT;

        // fetech extactly one 0-360 degrees' scan
        ans = me->drv->grabScanData(me->local_data, local_data_count);
        if (IS_OK(ans) || ans == RESULT_OPERATION_TIMEOUT)
            me->drv->ascendScanData(me->local_data, local_data_count);
        else
            printf("lidar error code: %d\n", ans);
        
        pthread_mutex_lock(&me->lidar_lock);
        memcpy(me->lidar_data, me->local_data, local_data_count * sizeof(rplidar_response_measurement_node_t));
        me->lidar_data_count = local_data_count;
        pthread_mutex_unlock(&me->lidar_lock);

        usleep(45000);
    }

    me->drv->stop();
    me->drv->stopMotor();

    RPlidarDriver::DisposeDriver(me->drv);

    free(me->lidar_data);
    free(me->local_data);
    
    printf("lidar quits.\n");
    return 0;
}

void RpLidar::init()
{
    lidar_data = (rplidar_response_measurement_node_t *) malloc(sizeof(rplidar_response_measurement_node_t) * MAX_LIDAR_DATA_COUNT);
    local_data = (rplidar_response_measurement_node_t *) malloc(sizeof(rplidar_response_measurement_node_t) * MAX_LIDAR_DATA_COUNT);
    if ((lidar_data == 0) || (local_data == 0) )
    {
      perror("mikes:lidar");
      printf("rplidar: insufficient memory\n");
      exit(1);
    }
    pthread_mutex_init(&lidar_lock, 0);
    if (!connect_lidar())
    {
      printf("connect lidar returned 0, exiting\n");
      exit(1);
    }
    program_runs = 1;
    if (pthread_create(&t, 0, lidar_thread, this) != 0)
    {
      perror("mikes:lidar");
      printf("creating thread for lidar\n");
    }
}

void RpLidar::stop() {
    program_runs = 0;
    pthread_join(t, NULL);
}


