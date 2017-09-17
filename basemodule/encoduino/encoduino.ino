// second arduino handles only reading from the encoders

#include <SoftwareSerial.h>

#define disable_tx() pinMode(10, INPUT);
//DDRB &= 0b11111011 
#define enable_tx() delayMicroseconds(5); DDRB |= 4; PORTB |= 4 

int16_t current_speedL, current_speedR;
int16_t stepL, stepR;

SoftwareSerial modem(9, 10);

void setup() 
{
  pinMode(9, INPUT);
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  modem.begin(19200);
  modem.listen();
  current_speedL = current_speedR = stepL = stepR = 0;
  pinMode(2, OUTPUT);  //data0
  pinMode(3, OUTPUT);  //data1
  pinMode(4, OUTPUT);  //data2
  pinMode(5, OUTPUT);  //data3
  pinMode(6, OUTPUT);  //data ready
  pinMode(7, INPUT);   //data ack
  digitalWrite(6, HIGH);
  pinMode(13, OUTPUT); // signal LED

  delay(300);
  encoders_clear_position();
  delay(300);
  encoders_reverse_motor1();
  delay(30);
}

uint8_t send_4bit(uint8_t b)
{
  b &= 15;
  b <<= 2;
  PORTD &= 0b11000011; // erase previous
  PORTD |= b;  // prepare new data0-data3
  PORTD &= 0b10111111;       // send data ready  
  uint16_t waiting = 0; // wait for ack
  while (((PIND & 128) == 128) && (waiting < 1000)) waiting++;
  PORTD |= 64;        // clear data ready, confirm ACK seen
  if (waiting == 1000)  // timeout, ack not received, cancel sending
    return 1;
  waiting = 0;        // wait for terminal ready
  while (((PIND & 128) == 0) && (waiting < 1000)) waiting++;
  if (waiting == 1000)  // timeout, terminal ready not received
    return 2;
  return 0;
}

uint8_t send_16bit(int16_t w)
{
  uint16_t uw = w;
  uint8_t ok = 0;
  ok |= send_4bit((uint8_t)uw);
  ok |= send_4bit((uint8_t)(uw >> 4));
  ok |= send_4bit((uint8_t)(uw >> 8));
  ok |= send_4bit((uint8_t)(uw >> 12));
  return ok;
}

uint8_t send_data()
{
  uint8_t ok = 0;
  ok |= send_16bit(current_speedL);
  ok |= send_16bit(current_speedR);
  ok |= send_16bit(stepL);
  ok |= send_16bit(stepR);
  return ok;
}

uint8_t byte_to_send;

void encoders_clear_position(void)
{
  byte_to_send = 0x28;
  modem.write(byte_to_send);
}

void encoders_reverse_motor1(void)
{
  byte_to_send = 0x32;
  modem.write(byte_to_send);  // reverse direction of motor 1
}

uint16_t query_position_right(void)
{
  uint16_t p;
  byte_to_send = 9;
  modem.write(byte_to_send);
  disable_tx();

  uint16_t waiting = 0;
  while ((modem.available() < 2) && (waiting < 50000)) 
  {
    waiting++;
    delayMicroseconds(1);
  }
  if (waiting >= 50000)
  {
    enable_tx();
    return 0x7FFF;
  } 
  p = modem.read() << 8;
  p += modem.read();
  enable_tx();
  return p;
}

uint16_t query_position_left(void)                                                                                                                                                                                                                                                                                                                   
{
  uint16_t p;
  byte_to_send = 10;
  modem.write(byte_to_send);
  disable_tx();
  uint16_t waiting = 0;
  while ((modem.available() < 2) && (waiting < 50000)) 
  {
    waiting++;
    delayMicroseconds(1);
  }
  if (waiting >= 50000)
  {
    enable_tx();
    return 0x7FFF; 
  }
  p = modem.read() << 8;
  p += modem.read();
  enable_tx();
  return p;
}

uint16_t query_speed_right(void)
{
  uint16_t p;
  byte_to_send = 17;
  modem.write(byte_to_send);
  disable_tx();
  uint16_t waiting = 0;
  while ((modem.available() < 2) && (waiting < 50000)) 
  {
    waiting++;
    delayMicroseconds(1);
  }
  if (waiting >= 50000)
  {
    enable_tx();
    return 0x7FFF; 
  }
  p = modem.read() << 8;
  p += modem.read();
  enable_tx();
  return p;
}

uint16_t query_speed_left(void)
{
  uint16_t p;
  byte_to_send = 18;
  modem.write(byte_to_send);
  disable_tx();
  uint16_t waiting = 0;
  while ((modem.available() < 2) && (waiting < 50000)) 
  {
    waiting++;
    delayMicroseconds(1);
  }
  if (waiting >= 50000)
  {
    enable_tx();
     return 0x7FFF;
  } 
  p = modem.read() << 8;
  p += modem.read();
  enable_tx();
  return p;
}

void read_encoders()
{
  current_speedL = query_speed_left();
  delay(1);
  current_speedR = query_speed_right();
  delay(1);
  stepL = query_position_left(); 
  delay(1);
  stepR = query_position_right();
  delay(1);
}

// A1 can send encoders reset request by pulling its Tx (our A5) down for 100ms
void check_position_reset()
{
  uint8_t position_reset_request = 0;
  for (int i = 0; i < 40; i++)
  {
    delay(1);
    if (analogRead(5) < 200) position_reset_request++;
  }
  if (position_reset_request == 40)
  {
    for (int i = 0; i < 70; i++)
      if (analogRead(5) < 200) position_reset_request++;

    if (position_reset_request == 110)
      encoders_clear_position();
    
    delay(10);
  }
}

void loop() 
{
  read_encoders();
  if (send_data())
  {
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13, LOW);    
  }
  check_position_reset();
}

