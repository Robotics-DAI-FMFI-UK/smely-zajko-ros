// obtains reading from 5x SRF08 sensors in background using I2C based on interrupts

#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

// ==============================
// addresses of SRF08 US sensors:
//
//
//      E8   E6    E4
//   EA               E2
//          front
//
// ==============================


// 7-bit I2C address of the front ultrasonic sensor  // 0xE6 == 115
#define SRF08_ADDR 115

extern volatile uint8_t THRESHOLD_OBSTACLE_SIDE;
extern volatile uint8_t THRESHOLD_OBSTACLE_LR;
extern volatile uint8_t THRESHOLD_OBSTACLE_M;

// initialize all ultrasonic ports
void init_ultrasonic();

// read one of the front ultrasonic sensors, adr=SRF08_ADDR - 2 ... SRF08_ADDR + 2
uint16_t srf08_sample(uint8_t adr);

// these are already internally setup from init_ultrasonic()
void srf08_set_gain(uint8_t adr, uint8_t gain);
void srf08_set_range(uint8_t adr, uint8_t range);

// service routine
void srf08_change_address(uint8_t adr, uint8_t newadr);

// samples all three rear ultrasonic, if any of them senses a critical distance, returns 1, otherwise returns 0
uint8_t obstacle_on_the_back();

#endif

