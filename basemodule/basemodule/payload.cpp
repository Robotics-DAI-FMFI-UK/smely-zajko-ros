#include <Arduino.h>

#define PAYLOAD_IR_SENSOR 6

uint8_t has_payload()
{
  if (analogRead(PAYLOAD_IR_SENSOR) > 500)
    return 1;
  return 0; 
}

