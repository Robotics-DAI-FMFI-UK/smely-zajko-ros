#include <Arduino.h>
#include "communication.h"
#include "steering.h"
#include "timers.h"
#include "remote_control.h"
#include "ultrasonic.h"

static char prnbuf[125];
static uint8_t data[16];
static uint8_t data_index;

uint8_t reset_steps;
uint8_t backing_up;
uint8_t payload_detected;

void init_communication()
{
  data_index = 0;
  backing_up = 0;
  Serial.begin(115200);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(13, OUTPUT); // data ack pin is output
  digitalWrite(13, HIGH);
  
  DDRC &= ~8;        // data ready pin is input  
  PCICR |= 2;        // enable PCINT1
  PCMSK1 = 8;        // turn on PCINT1 only for pin PCINT11 (PC3/ADC3) - data ready signal from another Arduino
}

void status_reporting()
{
  int32_t sumL = offsetL;
  sumL += stepL;
  int32_t sumR = offsetR;
  sumR += stepR;

  sprintf(prnbuf, "@%ld %ld %d %d %d %d %u %u %u %u %u %u %u %lu", 
          sumL, sumR, current_speedL, current_speedR, blocked,
          obstacle, dist[0], dist[1], dist[2], dist[3], dist[4], payload_detected, blocked_behind, tm);

  Serial.println(prnbuf);
}

// we use Tx pin to indicate to encoduino to reset the position encoders by sending at least 100ms low pulse
void encoders_clear_position()
{
//  Serial.end();
  UCSR0B &= 0b11110111;
  DDRD |= 0b00000010;
  PORTD &= 0b11111101;
  delay(120);
  PORTD |= 0b00000010;
  UCSR0B |= 0b00001000;
  //Serial.begin(115200);
//  while (Serial.read() != ';');
}

void process_char(char c)
{
  static char expect_command = 1;
  static char expect_value = 0;
  static int value;
  static char sign;
  static char command;

  if (c == ';') 
  {
    expect_command = 1;
    //Serial.println("!cmd");
    // execute the previous command
    switch (command)
    {
      case 's': if (value > MAXSPEED) value = MAXSPEED;
                value *= sign;
                if ((speed != value) && (!remote_override))
                {
                    //Serial.println("!go");
                    speed = value;
                    speed_req = 1;
                    if (speed < 0) backing_up = 1;
                    else backing_up = 0;
                }
                expect_value = 0;
                break;
      case 'd': if (value > 80) value = 80;
                value *= sign;
                if ((value != dir) && (!remote_override))
                { 
                  dir = value;
                  speed_req = 1;
                }
                expect_value = 0;
                break;  
      case 'c': THRESHOLD_OBSTACLE_LR = value;
                expect_value = 0;
                break;
        case 'b': THRESHOLD_OBSTACLE_M = value;
                  expect_value = 0;
                  break;
        case 'm': MAXSPEED = value;
                  expect_value = 0;
                  break;
    }
  }
  else if (expect_command)
  {
    switch (c)
    {
      case '/': command = '/';
                backing_up = 0;
                stop_now();
                break;
      case 's': command = 's';
                expect_command = 0;
                value = 0;
                expect_value = 1;
                sign = 1;
                break; 
        case 'd': command = 'd';
                  expect_command = 0;
                  value = 0;
                  expect_value = 1;
                  sign = 1;
                  break;
        case 'c': command = 'c';
                  expect_command = 0;
                  value = 0;
                  expect_value = 1;
                  break;
        case 'b': command = 'b';
                  expect_command = 0;
                  value = 0;
                  expect_value = 1;
                  break;
        case 'r': reset_steps = 1;
                  command = ' ';
                  break;
        case 'i': ignore_obstacles = 1;
                  break;
        case 'o': ignore_obstacles = 0;
                  break;
        case 'm': command = 'm';
                  expect_command = 0;
                  value = 0;
                  expect_value = 1;
                  break;
        default: command = ' ';
                 break;
    }
  }
  else if (expect_value)
  {
    if (c == '-') sign = -1;
    else if ((c >= '0') && (c <= '9'))
    {
      value *= 10;
      value += c - '0';
    }
  }
}

void execute_master_commands()
{
  while (Serial.available())
    process_char(Serial.read());
}

static uint32_t last_tm = 0;

// background communication with encoduino (second arduino reporting encoders state)
ISR(PCINT1_vect)  //data ready
{
  if (PINC & 8) digitalWrite(13, HIGH);   // ACK has been read, get ready for next chunk
  else                                     // new chunk arrives
  {
    if (tm - last_tm > 3) data_index = 0;  // first in a bunch? reset index
    last_tm = tm;
    data[data_index++] = PIND >> 4;       // store this chunk
    if (data_index == 17)                   // data complete, load them
    {
      current_speedL = -(int16_t)((data[3] << 12) | (data[2] << 8) | (data[1] << 4) | data[0]);
      current_speedR = -(int16_t)((data[7] << 12) | (data[6] << 8) | (data[5] << 4) | data[4]);
      stepL = -(int16_t)((data[11] << 12) | (data[10] << 8) | (data[9] << 4) | data[8]);
      stepR = -(int16_t)((data[15] << 12) | (data[14] << 8) | (data[13] << 4) | data[12]);
      payload_detected = data[16];
      data_index = 0;
    }
    digitalWrite(13, LOW);                 // ACK that the chunk was read
  }
}

