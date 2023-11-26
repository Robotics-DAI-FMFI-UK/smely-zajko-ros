#include "steering.h"
#include "ultrasonic.h"
#include "servo.h"
#include "communication.h"
#include "remote_control.h"
#include "emergency_stop.h"

#define POSITION_RESET_INTERVAL 15000
#define UPDATE_FREQUENCY 3

volatile char speed_req;
volatile char speed;
volatile char dir;

volatile int16_t current_speedL;
volatile int16_t current_speedR;

volatile int16_t stepL;
volatile int16_t stepR;

int32_t offsetL;
int32_t offsetR;

uint8_t blocked = 0;
uint8_t obstacle = 0;
uint8_t blocked_behind = 0;

volatile uint8_t ignore_obstacles = 0;

volatile int16_t pwm_speedL = 0;
volatile int16_t pwm_speedR = 0;

volatile int16_t target_speedL = 0;
volatile int16_t target_speedR = 0;

static volatile uint8_t next_updateL;
static volatile uint8_t next_updateR;

uint16_t dist[5];

void stop(void)
{
  slow_set_servo(0, PWM_STOP);
  target_speedL = 0;
  target_speedR = 0;
}

void stop_now(void)
{
  set_servo(0, PWM_STOP);
  target_speedL = 0;
  target_speedR = 0;
  pwm_speedL = PWM_STOP;
  pwm_speedR = PWM_STOP;
}

void stop_immediatelly(void)
{
  int16_t saveStepL = stepL;
  int16_t saveStepR = stepR;

  encoders_clear_position();
  
  offsetL += saveStepL;
  offsetR += saveStepR;
  stepL = 0;
  stepR = 0;

  target_speedL = 0;
  target_speedR = 0;

  speed = 0;
  
  pwm_speedL = PWM_STOP;
  pwm_speedR = PWM_STOP;
}

void init_steering()
{
  next_updateL = 0;
  next_updateR = 0;
  offsetL = (int32_t)0;
  offsetR = (int32_t)0;
  pwm_speedL = PWM_STOP;
  pwm_speedR = PWM_STOP;
  init_servo();
  set_servo(1, PWM_STOP);
  set_servo(2, PWM_STOP);
}

void compute_update_spd(void)
{
  if ((obstacle && (pwm_speedL > PWM_STOP) && (pwm_speedR > PWM_STOP) &&
      ((!ignore_obstacles) && 
       (!obstacle_override))) || 
      blocked) 
  return;
  
  if (dir >= 0)
  {
    target_speedL = 40;
    target_speedR = 40 - dir;
  }
  else
  {
    target_speedL = 40 + dir;
    target_speedR = 40;
  }
  target_speedL *= speed;
  target_speedL += 20;
  target_speedL /= 40;
  target_speedR *= speed;
  target_speedR += 20;
  target_speedR /= 40;

  //sprintf(prnbuf, "! spd %d %d\n\r", speedL, speedR);
  //Serial.println(prnbuf);

  speed_req = 0;

  //sprintf(prnbuf, "! SET  dir=%d, spd=%d, targetL=%d, targetR=%d,  realL=%d, realR=%d, currL=%d, currR=%d\n", (int)dir, (int)speed, target_speedL, target_speedR, pwm_speedL, pwm_speedR, current_speedL, current_speedR);
  //Serial.println(prnbuf);
}

void halt_request(void)
{ 
  if (blocked) return;

  stop_immediatelly();
  blocked = 1;
    
  Serial.println("!red switch pressed");
  delay(50);    
}

void unblock()
{
  Serial.println("!red_switch_released");
  delay(10);
  blocked = 0;
}

void forced_stop(void)
{
//  int backup_speedL = target_speedL;
//  int backup_speedR = target_speedR;

  Serial.println("!need a break");
  stop();
  delay(500); //wait until it acutally "stops"

  //Serial.println("!reset pos to 0");
  stop_immediatelly();

  offsetL += stepL;
  offsetR += stepR;

  current_speedL = 0;
  current_speedR = 0;
  //forced_print = 1;

//  target_speedL = backup_speedL;
//  target_speedR = backup_speedR;

   Serial.println("!resume");
//    sprintf(prnbuf, "!resume\n\r");
//    usart1_putstr();
}

void steering()
{
  if (emergency_stop_pressed()) halt_request();
  else if (blocked) unblock();       

  /* master requested reset of odometry */
  if (reset_steps)
  {
    offsetL = (int32_t)(-stepL);
    offsetR = (int32_t)(-stepR);
    reset_steps = 0;
    Serial.println("!reset");
  }

  /* forced stop and reset of encoders */    
  if ((abs(stepL) > POSITION_RESET_INTERVAL) || (abs(stepR) > POSITION_RESET_INTERVAL))
  {
    forced_stop();
  }

  if (backing_up)
  {
    if (obstacle_on_the_back()) 
    {
      stop_now();
      speed = 0;
      blocked_behind = 30;
      // Serial.println("!GUGUGUGU");
    }
    else if (blocked_behind) blocked_behind--;
  } else blocked_behind = 0;

  if (backing_up && blocked_behind) return;
  if (speed_req) compute_update_spd();        //REACT  
}


void update_real_spd()
{
  if (next_updateL) next_updateL--;
  else
  {
      if (target_speedL > current_speedL)
    {
       if (pwm_speedL < PWM_FWD) pwm_speedL++;
       next_updateL = UPDATE_FREQUENCY;
    }
    else if (target_speedL < current_speedL)
    {
      if (pwm_speedL > PWM_BWD) pwm_speedL--;
      next_updateL = UPDATE_FREQUENCY;
    }
  }

  if (next_updateR) next_updateR--;
  else
  {
    if (target_speedR > current_speedR)
    {
      if (pwm_speedR < PWM_FWD) pwm_speedR++;
      next_updateR = UPDATE_FREQUENCY;
    }
    else if (target_speedR < current_speedR)
    {
      if (pwm_speedR > PWM_BWD) pwm_speedR--;
      next_updateR = UPDATE_FREQUENCY;
    }
  }
         
  set_servo(2, pwm_speedR);
  set_servo(1, pwm_speedL);
}

void obstacle_avoidance(void)
{
  static uint8_t which_sensor = 0;      
  static int8_t seen_obstacle = 0;
  static uint8_t alternate = 0;

  if (!alternate)
  {
    dist[2] = srf08_sample(SRF08_ADDR);  // always sample center sensor
    if (dist[2] < 25) dist[2] = 200;
  }
  else
  {
    // and one of the other 4 (round robin)
    dist[which_sensor] = srf08_sample(SRF08_ADDR + which_sensor - 2);
  
  // umbrella solution
  //dist[which_sensor] = 200;
  
    if (dist[which_sensor] < 10) dist[which_sensor] = 200;
    which_sensor++;
  }
  alternate ^= 1;
  
  if (which_sensor == 2) which_sensor++;
  else if (which_sensor == 5) which_sensor = 0;
  
  //detect obstacle
  if ((dist[0] < THRESHOLD_OBSTACLE_SIDE) || (dist[1] < THRESHOLD_OBSTACLE_LR) || (dist[2] < THRESHOLD_OBSTACLE_M)
      || (dist[3] < THRESHOLD_OBSTACLE_LR) || (dist[4] < THRESHOLD_OBSTACLE_SIDE))
    seen_obstacle += 5;
  else 
  if (seen_obstacle) seen_obstacle --;

  if (seen_obstacle > 15)
  {
    seen_obstacle = 15;
    if ((!ignore_obstacles) && (!obstacle))
    {
      stop();
      obstacle = 1;
      Serial.println("!obs");
    }
  }
  else if ((!seen_obstacle) && (obstacle)) 
  {
    obstacle = 0;
    Serial.println("!clear");
    compute_update_spd();
  }
}

