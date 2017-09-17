#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

extern volatile int16_t MAXSPEED;
extern volatile uint8_t remote_override;
extern volatile uint8_t obstacle_override;

void init_remote_control();
void remote_control();

#endif

