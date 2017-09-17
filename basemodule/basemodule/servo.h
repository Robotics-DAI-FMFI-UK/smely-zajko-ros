#ifndef __SERVO_H__
#define __SERVO_H__

#define PWM_STOP  (3 * F_CPU / 8 / 4000)
#define PWM_BWD    (F_CPU / 8 / 2000)
#define PWM_FWD    (2 * F_CPU / 8 / 2000)

// initialize the servo module
void init_servo();

// set servo speed which=1/2, value=PWM_BWD...PWM_STOP...PWM_FWD
void set_servo(uint8_t which, uint16_t value);

// same but take it slowly (not to spill the beer?)
void slow_set_servo(unsigned char which, unsigned int value);

#endif

