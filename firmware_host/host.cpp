// Runs the real sketch (../mars_arm/mars_arm.ino) on a PC, for testing the ground station
// against the actual firmware logic instead of the Python imitation of it.
//
// It talks to groundstation/hostfw.py over stdin/stdout, one line each way per time slice:
//   in :  A <microseconds> <button 0|1> <A0> <A1> <serial bytes to the board, hex, or ->
//   out:  S <time us> <servo pulse us x4> <LED> <RX bytes dropped> <serial bytes from the board, hex, or ->
// The sketch only runs while it has been granted time, so tests are repeatable.
//
// What this does not model: the AVR's speed, its 2 KB of RAM, or 16-bit ints. The 64-byte
// serial receive buffer and the 115200 baud arrival rate are modelled, because a sketch
// that stops reading for too long really does lose bytes.
#include <ctype.h>

#include <deque>
#include <string>
#include <utility>

#include "Arduino.h"
#include "EEPROM.h"

static const uint64_t LOOP_COST_US = 100;   // virtual time one pass of loop() takes
static const uint64_t BYTE_US = 87;         // one byte at 115200 baud, 8N1
static const size_t RX_CAPACITY = 63;       // usable space in the Uno's 64-byte ring buffer

static uint64_t now_us = 0, grant_us = 0, last_arrival_us = 0;
static std::deque<std::pair<uint64_t, uint8_t>> rx_wire;   // bytes still on the wire: arrival time, value
static std::deque<uint8_t> rx_buffer;
static unsigned long rx_dropped = 0;
static std::string tx;
static int button = HIGH, pots[2] = {512, 512}, led = LOW;
int host_servo_us[20];
HostSerial Serial;
HostEEPROM EEPROM;
static const char *eeprom_file = NULL;

void host_eeprom_changed() {
  if (!eeprom_file) return;
  if (FILE *f = fopen(eeprom_file, "wb")) {
    fwrite(EEPROM.cells, 1, sizeof(EEPROM.cells), f);
    fclose(f);
  }
}

static void deliver() {
  while (!rx_wire.empty() && rx_wire.front().first <= now_us) {
    if (rx_buffer.size() < RX_CAPACITY) rx_buffer.push_back(rx_wire.front().second);
    else rx_dropped++;
    rx_wire.pop_front();
  }
}

static void sync() {
  printf("S %llu %d %d %d %d %d %lu ", (unsigned long long)now_us, host_servo_us[3], host_servo_us[5],
         host_servo_us[6], host_servo_us[9], led, rx_dropped);
  if (tx.empty()) putchar('-');
  for (unsigned char ch : tx) printf("%02x", ch);
  putchar('\n');
  fflush(stdout);
  tx.clear();

  static char in[4096];
  unsigned long long us;
  int consumed = 0;
  if (!fgets(in, sizeof(in), stdin) || sscanf(in, "A %llu %d %d %d %n", &us, &button, &pots[0], &pots[1], &consumed) < 4)
    exit(0);
  button = button ? LOW : HIGH;             // the button pulls the pin to ground
  grant_us = now_us + us;
  for (const char *p = in + consumed; isxdigit(p[0]) && isxdigit(p[1]); p += 2) {
    char byte[3] = {p[0], p[1], 0};
    last_arrival_us = (last_arrival_us > now_us ? last_arrival_us : now_us) + BYTE_US;
    rx_wire.push_back({last_arrival_us, (uint8_t)strtoul(byte, NULL, 16)});
  }
}

static void advance(uint64_t us) {
  uint64_t target = now_us + us;
  while (now_us < target) {
    if (now_us >= grant_us) sync();
    now_us = target < grant_us ? target : grant_us;
  }
}

unsigned long millis() { return (unsigned long)(now_us / 1000); }
void delay(unsigned long ms) { advance((uint64_t)ms * 1000); }
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t pin, uint8_t value) { if (pin == 13) led = value; }
int digitalRead(uint8_t pin) { return pin == 2 ? button : HIGH; }
int analogRead(uint8_t pin) { return pin == A0 ? pots[0] : pin == A1 ? pots[1] : 0; }
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int HostSerial::available() { deliver(); return (int)rx_buffer.size(); }
int HostSerial::read() {
  deliver();
  if (rx_buffer.empty()) return -1;
  int value = rx_buffer.front();
  rx_buffer.pop_front();
  return value;
}
void HostSerial::print(const char *s) { tx += s; }
void HostSerial::print(char c) { tx += c; }
void HostSerial::print(long n) { tx += std::to_string(n); }
void HostSerial::print(unsigned long n) { tx += std::to_string(n); }
void HostSerial::print(double value, int digits) {   // same steps as the Arduino core's Print::printFloat
  float number = (float)value;
  if (isnan(number)) return print("nan");
  if (isinf(number)) return print("inf");
  if (number > 4294967040.0f || number < -4294967040.0f) return print("ovf");
  if (number < 0.0f) { print('-'); number = -number; }
  float rounding = 0.5f;
  for (int i = 0; i < digits; i++) rounding /= 10.0f;
  number += rounding;
  unsigned long whole = (unsigned long)number;
  float rest = number - (float)whole;
  print(whole);
  if (digits > 0) print('.');
  while (digits-- > 0) {
    rest *= 10.0f;
    unsigned int digit = (unsigned int)rest;
    print((unsigned long)digit);
    rest -= digit;
  }
}

#include "../mars_arm/mars_arm.ino"

int main(int argc, char **argv) {
  if (argc > 1) {
    eeprom_file = argv[1];
    if (FILE *f = fopen(eeprom_file, "rb")) {
      if (fread(EEPROM.cells, 1, sizeof(EEPROM.cells), f) != sizeof(EEPROM.cells)) memset(EEPROM.cells, 0xFF, sizeof(EEPROM.cells));
      fclose(f);
    }
  }
  setup();
  for (;;) {
    loop();
    advance(LOOP_COST_US);
  }
}
