#include "emergency_stop.h"

// A7 -> 10k pullup to 5V, switch pressed connects with GND
uint8_t emergency_stop_pressed()
{
  uint16_t red_switch = analogRead(7);
  if (red_switch < 200) return 1;
  else return 0;
}
