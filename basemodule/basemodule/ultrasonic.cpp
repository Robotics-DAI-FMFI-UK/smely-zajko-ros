// obtains reading from 5x SRF08 sensors in background using I2C based on interrupts

#include <Arduino.h>
#include <Wire.h>

#include "ultrasonic.h"

#define CRITICAL_REAR_DISTANCE_WHILE_BACKING_UP_CM 18

volatile uint8_t THRESHOLD_OBSTACLE_SIDE = 25;
volatile uint8_t THRESHOLD_OBSTACLE_LR = 30;
volatile uint8_t THRESHOLD_OBSTACLE_M = 80;

int16_t srf08_echo;

uint8_t old_echo;
volatile uint8_t us_mask;

uint8_t us_first_phase;
uint32_t tm_ultrasonic1, tm_ultrasonic2;

ISR(PCINT0_vect)  //rear ultrasonic sensors
{
  uint8_t change = (PINB ^ old_echo) & us_mask;
  old_echo = PINB;
  if (change)
  {
    if (us_first_phase) 
    {
      tm_ultrasonic1 = micros();
      us_first_phase = 0;
    }
    else tm_ultrasonic2 = micros();
  }
}

void unstuck_us(uint8_t which)
{
    switch (which) {
      case 1:  
        DDRB |= 1;  // US1 ECHO
        PORTB |= 1; // reset the guy over its ECHO pin
        delayMicroseconds(200);
        PORTB &= 0b11111110;
        DDRB &= 0b11111110;
        break;
      case 2:
        DDRB |= 8;  // US2 ECHO
        PORTB |= 8; // reset the guy over its ECHO pin
        delayMicroseconds(200);
        PORTB &= 0b11110111;
        DDRB &= 0b11110111;
        break;
      case 3:
        DDRB |= 16;  // US3 ECHO
        PORTB |= 16; // reset the guy over its ECHO pin
        delayMicroseconds(200);
        PORTB &= 0b11101111;
        DDRB &= 0b11101111;
        break;
    } 
}

uint8_t wait_for_echo_and_read_it(uint8_t which)
{
    uint32_t timeout = millis() + 30;
    while ((tm_ultrasonic2 == 0) && (millis() < timeout))
      delay(1);
    if (millis() < timeout)
    {
      uint32_t dist = (tm_ultrasonic2 - tm_ultrasonic1) / 58;
      if (dist < CRITICAL_REAR_DISTANCE_WHILE_BACKING_UP_CM)
      {
        // Serial.print("! "); Serial.print(which); Serial.print(" "); Serial.println(dist);
        return 1;  // return 0 to not use them
      }      
    }
    else unstuck_us(which);
    return 0;
}

void trigger_rear_us(uint8_t which)
{
  uint8_t trig;
  switch (which) {
    case 1: trig = 4; us_mask = 1; break;
    case 2: trig = 2; us_mask = 8; break;
    case 3: trig = 1; us_mask = 16; break;
  }

  old_echo = PINB;
  us_first_phase = 1;
  tm_ultrasonic2 = 0;

  PORTC |= trig;
  delayMicroseconds(10);
  PORTC &= ~trig;
}

uint8_t obstacle_on_the_back()
{
  uint8_t rv = 0;
  us_first_phase = 1;
  tm_ultrasonic2 = 0;
  
  // enable rear ultrasonic interrupt
  PCMSK0 |= (1 | 8 | 16);   // PCINT0, PCINT3, PCINT4 => echo signals
  PCICR |= 1;   // PCIE0 group on

  for (uint8_t which = 1; which <= 3; which++)
  {
    trigger_rear_us(which);
    rv = wait_for_echo_and_read_it(which);
    if (rv) break;
    if (which < 3) delay(5);
  }
  // clear rear ultrasonic interrupt
  PCICR &= 0b11111110;
  return rv;
}

void init_ultrasonic()
{
  // 3x rear ultrasonic
  pinMode(8, INPUT);   //us1 echo
  pinMode(11, INPUT);  //us2 echo
  pinMode(12, INPUT);  //us3 echo
  DDRC |= 7;  //us1, us2, us3 trig
  PORTC &= 11111000;
  
  DDRC &= ~(16 | 32);
  PORTC |= (16 | 32);
  Wire.begin();
  delay(700);
  for (int i = -2; i < 2; i++)
  {
    srf08_set_range(SRF08_ADDR + i, (uint8_t)60); // range is (X * 43 + 43) [mm], i.e. X=60 => 2623 mm
    srf08_set_gain(SRF08_ADDR + i, (uint8_t)9);  // gain 9 corresponds to 128/1025
    delay(3);
  } 
}

//set the gain register (values 0-31, default is maximum gain 31)
void srf08_set_gain(uint8_t adr, uint8_t gain)
{
  uint8_t buf[2];

  buf[0] = 1;  // gain register
  buf[1] = gain;
  Wire.beginTransmission(adr);
  Wire.write(buf, 2);
  Wire.endTransmission();
}

//Set the range register (0-255, default is maximum range 255 (11 metres))
//this controls how long time it takes to complete ranging, i.e. decreasing this value is a good idea.
//The range is ((Range Register x 43mm) + 43mm)
//The maximum real range is 6m (value 140). Range of 1m is value 24.
//If you set the range lower, you probably should decrease the gain.
void srf08_set_range(uint8_t adr, uint8_t range)
{
  uint8_t buf[2];

  buf[0] = 2;  // range register
  buf[1] = range;
  Wire.beginTransmission(adr);
  Wire.write(buf, 2);
  Wire.endTransmission();
}

uint16_t srf08_sample(uint8_t adr)
{
  uint8_t buf[2];
  srf08_echo = 255;

  buf[0] = 0;     // command register
  buf[1] = 0x51;  // Real Ranging Mode - Result in centimeters
  Wire.beginTransmission(adr);
  Wire.write(buf, 2);
  Wire.endTransmission();
  
  delay(70);

  buf[0] = 2; 
  Wire.beginTransmission(adr);
  Wire.write(buf, 1);
  Wire.endTransmission();
  Wire.requestFrom(adr, (uint8_t)2);
  buf[0] = Wire.read();
  buf[1] = Wire.read();
  return (uint16_t)buf[1] | ((uint16_t)buf[0] << 8);
}

void srf08_change_address(uint8_t adr, uint8_t newadr)
{
  uint8_t buf[2];
  buf[0] = 0;     // command register
  buf[1] = 0xA0;  // 1st in sequence to change I2C address
  Wire.beginTransmission(adr);
  Wire.write(buf, 2);
  Wire.endTransmission();
  buf[1] = 0xAA;  // 2nd in sequence to change I2C address
  Wire.beginTransmission(adr);
  Wire.write(buf, 2);
  Wire.endTransmission();
  buf[1] = 0xA5;  // 3rd in sequence to change I2C address
  Wire.beginTransmission(adr);
  Wire.write(buf, 2);
  Wire.endTransmission();
  buf[1] = newadr;
  Wire.beginTransmission(adr);
  Wire.write(buf, 2);
  Wire.endTransmission();
}

