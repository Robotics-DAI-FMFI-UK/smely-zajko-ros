#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

extern uint8_t reset_steps;
extern uint8_t backing_up;

void init_communication();
void status_reporting();
void execute_master_commands();
void encoders_clear_position();

#endif

