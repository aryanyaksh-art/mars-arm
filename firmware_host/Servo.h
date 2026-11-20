// Stand-in for the Arduino Servo library: remembers the last pulse width sent to each pin.
#pragma once
#include <stdint.h>

extern int host_servo_us[20];

class Servo {
 public:
  uint8_t attach(int pin) { pin_ = pin; return 1; }
  void writeMicroseconds(int us) { if (pin_ >= 0 && pin_ < 20) host_servo_us[pin_] = us; }
  void write(int angle) { writeMicroseconds(544 + (long)angle * (2400 - 544) / 180); }
 private:
  int pin_ = -1;
};
