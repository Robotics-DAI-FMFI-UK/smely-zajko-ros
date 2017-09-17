#include <Arduino.h>
#include "remote_control.h"
#include "steering.h"

// CH1 -> D2
// CH3 -> D3

volatile int32_t vypis;

static uint32_t last_t1, last_t2;

volatile uint8_t remote_override = 0;
volatile uint8_t obstacle_override = 0;
volatile uint8_t extra_flag = 0;
volatile int16_t MAXSPEED = 10;

void init_remote_control()
{
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  // INT0 and INT1 generate interrupt on signal change
  EICRA = 0b00000101;
  EIMSK = 0b00000011;
  vypis = 0;
  // remote_override used to be detecting remote control channel5, but we do not have 
  // it connected, so it is kept on while signals are coming.
  // when transmitter is off, we get no signal anyway
  remote_override = 0;  
  // obstacle_override used to be connected to channel6 of remote control
  // and used to switch off obstacle avoidance when in indoor tricky environments, no need
  obstacle_override = 0;
}
 
ISR(INT1_vect)  //receiver fwd/bwd
{
  int32_t tm;
  uint32_t t1 = micros();
  tm = t1 - last_t1;
  last_t1 = t1;

  remote_override = 250;
  if ((PIND & 8) == 0)
  {
    tm -= 1500;
    if (tm > 500) tm = 0;
    if (tm < -500) tm = 0;
    int new_speed = tm * MAXSPEED / 500; 
    if (new_speed != speed)
    {
      speed = new_speed;
      speed_req = 1;
    }
  }
}

ISR(INT0_vect)  //receiver left/right
{
  int32_t tm;
  uint32_t t2 = micros();
  tm = t2 - last_t2;
  last_t2 = t2;

  remote_override = 250;
  if ((PIND & 4) == 0)
  {
    tm -= 1500;
    if (tm > 400) tm = 0;
    if (tm < -400) tm = 0;        
    int new_dir = tm / 10;
    if (new_dir != dir)
    {
      dir = new_dir;
      speed_req = 1;
    }
  }
}



