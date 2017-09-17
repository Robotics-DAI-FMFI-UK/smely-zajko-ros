#include <Arduino.h>
#include "timers.h"
#include "steering.h"
#include "remote_control.h"

volatile uint32_t tm;

void init_timer()
{
  tm = 0;
  //mega328 Timer2 setting:
  //mode 2 = CTC
  //prescaler = 1024
  //timer frequency = 100.16026 Hz
  //timer period = 0.009984 s
  //enable compare match A interrupt
  TCNT2 = 0; 
  TCCR2A = 0b00000010;
  TCCR2B = 0b00000111;
  OCR2A = 155;
  TIMSK2 = 0b00000010;  
}

// about 100-times per second
ISR(TIMER2_COMPA_vect)
{
  tm++;
  update_real_spd();
  if (remote_override) remote_override--;
}

