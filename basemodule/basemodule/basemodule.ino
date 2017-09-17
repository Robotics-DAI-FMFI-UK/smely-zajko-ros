#include "ultrasonic.h"
#include "communication.h"
#include "remote_control.h"
#include "steering.h"
#include "timers.h"

void setup() 
{
  init_steering();
  init_communication();
  init_timer();
  init_ultrasonic();
  init_remote_control();
}

void loop() 
{
  obstacle_avoidance();
  execute_master_commands();
  steering();
  status_reporting();
}


