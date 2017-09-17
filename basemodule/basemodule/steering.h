#ifndef __STEERING_H__
#define __STEERING_H__

#include <Arduino.h>

extern volatile char speed_req;
extern volatile char speed;
extern volatile char dir;

extern volatile int16_t current_speedL;
extern volatile int16_t current_speedR;

extern volatile int16_t stepL;
extern volatile int16_t stepR;

extern int32_t offsetL;
extern int32_t offsetR;

extern uint8_t blocked;
extern uint8_t obstacle;
extern uint8_t blocked_behind;

extern volatile uint8_t ignore_obstacles;

extern uint16_t dist[5];

void init_steering();
void steering();
void obstacle_avoidance();
void stop_now();
void update_real_spd();

#endif


