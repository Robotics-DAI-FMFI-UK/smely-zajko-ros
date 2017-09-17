#include <Arduino.h>
#include "servo.h"

static unsigned int srvo[3];

void init_servo()
{  
  // mega328 Timer1 setting:
  // mode 8 = PWM, Phase and Frequency Correct
  // Clear OC1A/OC1B on Compare Match when upcounting. Set OC1A/OC1B on Compare Match when downcounting.
  // prescaler 8  
  // timer period = 20ms
  // no interrupts
  srvo[1] = PWM_STOP;
  srvo[2] = PWM_STOP;

  OCR1A = PWM_STOP;
  OCR1B = PWM_STOP;
  TCCR1A = 0b10100000;
  TCCR1B = 0b00010010;
  TCCR1C = 0;
  ICR1 = F_CPU / 8 / 50 / 2;
  TIMSK1 = 0;
  TIFR1 = 0;
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
}

void set_servo(uint8_t which, uint16_t value)
{
  if (which == 1) OCR1A = value;
  else OCR1B = value;
  srvo[which] = value;
}

void slow_set_servo(unsigned char which, unsigned int value)
{
  if (which == 0)  // case 2 servos only
  {
    int delta2 = (srvo[2]<value)?1:-1;
    int delta3 = (srvo[3]<value)?1:-1;
    int d2 = abs(srvo[2] - value);
    int d3 = abs(srvo[3] - value);
    int d;
    long i;
    if (d2 > d3) d = d3;
    else d = d2;
    int s2 = srvo[2];
    int s3 = srvo[3];
    for (i = 0; i < d; i++)
    {
      long news = s2 + delta2 * i * d2 / d;
      set_servo(2, (int)news);
      news = s3 + delta3 * i * d3 / d;
      set_servo(3, (int)news);
      delayMicroseconds(50);
    } 
    set_servo(0, value);
  }
  else 
  {
    int delta = srvo[which]<value?1:-1;
  
    while (srvo[which] != value)
    {
      set_servo(which, srvo[which] + delta);
      delayMicroseconds(50);
    }
    set_servo(which, value);
  }
}

