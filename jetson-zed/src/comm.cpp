#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <time.h>
#include <math.h>

#include "../../src/robot_control/src/netutil.h"

#include "comm.h"



int connected = 0;
int reconnect_counter = 0;

static int depth_socket;

int init_comm()
{
    do {
      depth_socket = connect_to_server(ZAJKO_IP, ZAJKO_POSITION_AND_DEPTH_PORT);
      if (depth_socket == 0)
      {
        printf("connect to zajko depth server failed, retrying...\n");
        sleep(1);
        continue;
      }
      connected = 1;
    } while (!connected);
    return 1;
}

void close_conn()
{
	close(depth_socket);
}

void try_reconnecting()
{
  reconnect_counter = 0;
  depth_socket = connect_to_server(ZAJKO_IP, ZAJKO_POSITION_AND_DEPTH_PORT);
  if (depth_socket == 0)
  printf("reconnecting failed, will retry...\n");
  else 
  {
    printf("reconnected\n");
    connected = 1;
  }
}

static uint8_t depth_header[4];

int send_data(uint8_t *data)
{
	while (!connected) 
	{
      try_reconnecting();
      if (!connected) sleep(1);
	}

/* //semigraphic printout of sending packet	
 
	for (int i = 0; i < 60; i++)
		{
			for (int j = 0; j < 60; j++)
				printf("%c", data[i*60 + j]?'#':'.');
                        printf("\n");
		}
		*/

    data[0] = DEPTH_PACKET_TYPE;
    if (!send_packet(depth_socket, data, 3604))
    {
      close(depth_socket);
      connected = 0;
      printf("could not sent dpth. will try reconnecting next time...\n");
    }
    return 1;
}

