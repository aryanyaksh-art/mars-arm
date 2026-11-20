// Stand-in for the Arduino EEPROM library: 1 KB of memory that starts out erased (all 0xFF).
// If host.cpp was started with a file name, the memory is loaded from and saved to that file,
// so a test can "power-cycle" the board and check what it remembers.
#pragma once
#include <stdint.h>
#include <string.h>

void host_eeprom_changed();

struct HostEEPROM {
  uint8_t cells[1024];
  HostEEPROM() { memset(cells, 0xFF, sizeof(cells)); }
  uint8_t read(int at) { return cells[at]; }
  void update(int at, uint8_t value) { if (cells[at] != value) { cells[at] = value; host_eeprom_changed(); } }
  void write(int at, uint8_t value) { update(at, value); }
  template <typename T> T &get(int at, T &value) { memcpy(&value, cells + at, sizeof(T)); return value; }
  template <typename T> const T &put(int at, const T &value) {
    memcpy(cells + at, &value, sizeof(T));
    host_eeprom_changed();
    return value;
  }
};
extern HostEEPROM EEPROM;
