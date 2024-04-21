#ifndef _ZEDCOMM_H_
#define _ZEDCOMM_H_


#include "../../src/robot_control/src/util.h"


#define ZAJKO_IP                       "169.254.0.100"
#define ZAJKO_POSITION_AND_DEPTH_PORT  9772

#define DEPTH_REPORT_PERIOD            50    // 20 Hz
#define POSITION_REPORT_PERIOD         50    // 20 Hz
#define POSITION_PRINT_PERIOD          500   // 2 Hz
#define POSITION_PACKET_TYPE	       3
#define DEPTH_PACKET_TYPE	       	   4

#define RECONNECT_FREQUENCY            70


int init_comm();
void close_conn();

int send_data(uint8_t *data);


#endif
