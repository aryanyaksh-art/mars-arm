// Stand-in for the Arduino core, so mars_arm.ino can be compiled and run on a PC.
// Only what the sketch uses. Time is virtual: it advances when the sketch calls
// delay() and by a small fixed cost per loop() pass, and never faster than the
// test harness on the other end of stdin/stdout allows (see host.cpp).
#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define A0 14
#define A1 15
#define F(s) (s)
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define degrees(rad) ((rad) * 57.29577951308232)
#define radians(deg) ((deg) * 0.017453292519943295)

unsigned long millis();
void delay(unsigned long ms);
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
long map(long x, long in_min, long in_max, long out_min, long out_max);

struct HostSerial {
  void begin(unsigned long) {}
  int available();
  int read();
  void print(const char *s);
  void print(char c);
  void print(unsigned char n) { print((unsigned long)n); }
  void print(int n) { print((long)n); }
  void print(unsigned int n) { print((unsigned long)n); }
  void print(long n);
  void print(unsigned long n);
  void print(double n, int digits = 2);
  template <typename T> void println(T v) { print(v); print("\r\n"); }
  void println(double n, int digits) { print(n, digits); print("\r\n"); }
  void println() { print("\r\n"); }
};
extern HostSerial Serial;
