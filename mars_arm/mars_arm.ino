// Mars sample-collection arm: firmware for Arduino Uno + 4 hobby servos.
// Runs unchanged in Wokwi and on real hardware (set SIM_POTS to 0 for real).
//
// Serial protocol, 115200 baud, one command per line:
//   P x y        pick the sample at table position (mm) and drop it in the bin
//   G x y z      move the gripper tip to x y z (mm)
//   J b s e g    raw servo angles in degrees (base shoulder elbow gripper)
//   O / C        open / close gripper
//   H            home pose
//   X            abort the current motion
//   M ms         Mars comms delay: later commands wait ms before they arrive
//   Z b s e      servo trim offsets in degrees (base shoulder elbow); Z alone prints the trim
//   K b s e      servo trim gains in percent
//   ?            help
// Replies: "OK ...", "ERR ...", "# comment", and telemetry "T,b,s,e,g,phase"
// (four times a second while moving, once a second when idle).
//
// Table frame: origin at the base axis on the table, x forward, y left, z up.

#include <EEPROM.h>
#include <Servo.h>

#define SIM_POTS 1  // 1 = Wokwi sliders on A0/A1 set the demo rock position

const uint8_t PIN_BASE = 3, PIN_SHOULDER = 5, PIN_ELBOW = 6, PIN_GRIP = 9;
const uint8_t PIN_BTN = 2, PIN_LED = 13;

// Arm geometry in mm. These match the printed micro-servo arm in cad/build_parts.py.
const float L1 = 80.0;       // shoulder axis to elbow axis
const float L2 = 85.0;       // elbow axis to the middle of the jaws
const float BASE_H = 46.2;   // table to shoulder axis
const float GRIP_Y = -16.5;  // sideways offset of the jaws from the arm's plane (+ is left)

// Servo calibration: servo angle = ZERO + DIR * joint angle. If a joint moves
// the wrong way on your build, flip its DIR; if it is offset, change its ZERO.
const float BASE_DIR = 1;
const float SHOULDER_ZERO = 0, SHOULDER_DIR = 1;  // joint angle: upper arm above horizontal
const float ELBOW_ZERO = 0, ELBOW_DIR = 1;        // joint angle: between upper arm and forearm
const float GRIP_OPEN = 60.0, GRIP_CLOSED = 100.0;

// Joint limits in degrees, so the arm never folds into itself or the table.
const float UPPER_MIN = -10, UPPER_MAX = 120, INNER_MIN = 45, INNER_MAX = 150;

const float HOME_POSE[4] = {90, SHOULDER_ZERO + SHOULDER_DIR * 90, ELBOW_ZERO + ELBOW_DIR * 70, GRIP_OPEN};
const float Z_HOVER = 45, Z_GRIP = 14, Z_CARRY = 75;  // carry height clears the bin rim
const float BIN_X = 50, BIN_Y = -140;

const float STEP_DEG = 1.0;    // joint interpolation resolution
const uint8_t STEP_MS = 12;    // ~80 deg/s
const float SEGMENT_MM = 10;   // straight-line moves are cut into these

// Servo trim. No hobby servo goes exactly where it is told: the horn only fits in steps of
// about 17 degrees, and its travel is a few percent more or less than 180. A real joint ends up at
//   angle = 90 + (commanded - 90) * (1 + gain) + offset
// so writeServo() commands the angle that cancels that. The ground station measures the
// offsets and gains with its camera (key T) and stores them here with Z and K; they are kept
// in EEPROM, so they survive power-off.
float trimOffset[3] = {0, 0, 0};  // degrees: base, shoulder, elbow
float trimGain[3] = {0, 0, 0};    // percent
const uint8_t TRIM_SAVED = 0x5A;  // marks an EEPROM that holds a trim

Servo servos[4];
const uint8_t SERVO_PINS[4] = {PIN_BASE, PIN_SHOULDER, PIN_ELBOW, PIN_GRIP};
float cur[4];                  // current servo angles
float tip[3];                  // last commanded tip position
bool tipKnown = false;
const char *phase = "IDLE";

// Commands wait here until their Mars delay has elapsed.
const uint8_t LINE_MAX = 32;   // longest command, including its terminator
struct Pending { char text[LINE_MAX]; unsigned long due; bool used; };
Pending queue[4];
char line[LINE_MAX];
uint8_t lineLen = 0;
bool lineTooLong = false;
unsigned long marsDelay = 0;
unsigned long lastTelemetry = 0;

void writeServo(uint8_t i) {
  float angle = cur[i];
  if (i < 3) angle = 90 + (angle - 90 - trimOffset[i]) / (1 + trimGain[i] / 100);
  angle = constrain(angle, 0, 180);
  servos[i].writeMicroseconds(544 + (int)(angle * (2400.0 - 544.0) / 180.0));
}

void printTrim() {
  Serial.print(F("# trim"));
  for (uint8_t i = 0; i < 3; i++) { Serial.print(' '); Serial.print(trimOffset[i], 2); }
  Serial.print(F(" gain"));
  for (uint8_t i = 0; i < 3; i++) { Serial.print(' '); Serial.print(trimGain[i], 2); }
  Serial.println();
}

void loadTrim() {
  if (EEPROM.read(0) != TRIM_SAVED) return;
  EEPROM.get(1, trimOffset);
  EEPROM.get(1 + sizeof(trimOffset), trimGain);
  for (uint8_t i = 0; i < 3; i++) {  // a corrupted EEPROM must not throw the arm around
    if (!(fabs(trimOffset[i]) <= 20) || !(fabs(trimGain[i]) <= 15)) {
      for (uint8_t j = 0; j < 3; j++) trimOffset[j] = trimGain[j] = 0;
      return;
    }
  }
}

void saveTrim() {
  EEPROM.update(0, TRIM_SAVED);
  EEPROM.put(1, trimOffset);
  EEPROM.put(1 + sizeof(trimOffset), trimGain);
}

void telemetry(bool force) {
  if (!force && millis() - lastTelemetry < 250) return;
  lastTelemetry = millis();
  Serial.print(F("T,"));
  for (uint8_t i = 0; i < 4; i++) { Serial.print(cur[i], 1); Serial.print(','); }
  Serial.println(phase);
}

void enqueue(const char *text) {
  if (text[0] == 'M') {  // ground-station setting, applies immediately
    marsDelay = strtoul(text + 1, NULL, 10);
    Serial.print(F("OK comms delay ms=")); Serial.println(marsDelay);
    return;
  }
  for (uint8_t i = 0; i < 4; i++) {
    if (queue[i].used) continue;
    strncpy(queue[i].text, text, sizeof(queue[i].text) - 1);
    queue[i].text[sizeof(queue[i].text) - 1] = 0;
    queue[i].due = millis() + marsDelay;
    queue[i].used = true;
    if (marsDelay) {
      Serial.print(F("# uplink '")); Serial.print(text);
      Serial.print(F("' arrives in ms=")); Serial.println(marsDelay);
    }
    return;
  }
  Serial.println(F("ERR queue full"));
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineTooLong) Serial.println(F("ERR line too long"));  // never act on a cut-off command
      else if (lineLen) { line[lineLen] = 0; enqueue(line); }
      lineLen = 0;
      lineTooLong = false;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    } else {
      lineTooLong = true;
    }
  }
}

// Index of the arrived command that is due first, or -1.
int8_t nextArrived() {
  int8_t best = -1;
  unsigned long now = millis();
  for (uint8_t i = 0; i < 4; i++) {
    if (!queue[i].used || (long)(now - queue[i].due) < 0) continue;
    if (best < 0 || (long)(queue[i].due - queue[best].due) < 0) best = i;
  }
  return best;
}

bool abortArrived() {
  pollSerial();
  unsigned long now = millis();
  for (uint8_t i = 0; i < 4; i++) {
    if (queue[i].used && queue[i].text[0] == 'X' && (long)(now - queue[i].due) >= 0) {
      queue[i].used = false;
      return true;
    }
  }
  return false;
}

// Interpolate all four servos to the target together. False if aborted.
bool moveTo(const float target[4]) {
  float start[4], maxDelta = 0;
  for (uint8_t i = 0; i < 4; i++) {
    start[i] = cur[i];
    maxDelta = max(maxDelta, (float)fabs(target[i] - cur[i]));
  }
  int steps = max(1, (int)(maxDelta / STEP_DEG));
  for (int s = 1; s <= steps; s++) {
    for (uint8_t i = 0; i < 4; i++) {
      cur[i] = start[i] + (target[i] - start[i]) * s / steps;
      writeServo(i);
    }
    delay(STEP_MS);
    telemetry(false);
    if (abortArrived()) {
      Serial.println(F("# ABORT"));
      tipKnown = false;
      return false;
    }
  }
  return true;
}

// Inverse kinematics: tip position (mm) to base/shoulder/elbow servo angles.
bool solveIK(float x, float y, float z, float out[3]) {
  float r = sqrt(x * x + y * y);
  if (r <= fabs(GRIP_Y) + 1) return false;
  float yaw = atan2(y, x) - asin(GRIP_Y / r);  // turn so the offset jaws land on the target
  float rp = sqrt(r * r - GRIP_Y * GRIP_Y);    // reach within the arm's vertical plane
  float zw = z - BASE_H;
  float d = sqrt(rp * rp + zw * zw);
  if (d > L1 + L2 - 2 || d < fabs(L1 - L2) + 2) return false;
  float upper = degrees(atan2(zw, rp) + acos((L1 * L1 + d * d - L2 * L2) / (2 * L1 * d)));
  float inner = degrees(acos((L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2)));
  if (upper < UPPER_MIN || upper > UPPER_MAX || inner < INNER_MIN || inner > INNER_MAX) return false;
  out[0] = 90 + BASE_DIR * degrees(yaw);
  out[1] = SHOULDER_ZERO + SHOULDER_DIR * upper;
  out[2] = ELBOW_ZERO + ELBOW_DIR * inner;
  for (uint8_t i = 0; i < 3; i++) if (out[i] < 0 || out[i] > 180) return false;
  return true;
}

// Move the tip in a straight line to x y z. False if unreachable or aborted.
bool lineTo(float x, float y, float z) {
  float from[3] = {x, y, z};
  uint8_t segments = 1;
  if (tipKnown) {
    float dx = x - tip[0], dy = y - tip[1], dz = z - tip[2];
    segments = max(1, (int)ceil(sqrt(dx * dx + dy * dy + dz * dz) / SEGMENT_MM));
    from[0] = tip[0]; from[1] = tip[1]; from[2] = tip[2];
  }
  for (uint8_t s = 1; s <= segments; s++) {
    float f = (float)s / segments, target[4];
    if (!solveIK(from[0] + (x - from[0]) * f, from[1] + (y - from[1]) * f,
                 from[2] + (z - from[2]) * f, target)) {
      Serial.println(F("ERR unreachable"));
      return false;
    }
    target[3] = cur[3];
    if (!moveTo(target)) return false;
  }
  tip[0] = x; tip[1] = y; tip[2] = z;
  tipKnown = true;
  return true;
}

// Swing to x y z in joint space (an arc around the base). Used for long
// transfers, where a straight line could cut too close to the base to reach.
bool swingTo(float x, float y, float z) {
  tipKnown = false;
  return lineTo(x, y, z);
}

bool grip(float angle) {
  float target[4] = {cur[0], cur[1], cur[2], angle};
  bool ok = moveTo(target);
  if (ok) delay(200);  // let the jaws settle
  return ok;
}

bool goHome() {
  phase = "HOME";
  tipKnown = false;
  return moveTo(HOME_POSE);
}

bool pick(float x, float y) {
  float probe[3];
  if (!solveIK(x, y, Z_GRIP, probe) || !solveIK(x, y, Z_HOVER, probe) || !solveIK(x, y, Z_CARRY, probe)) {
    Serial.println(F("ERR target out of reach"));
    return false;
  }
  Serial.print(F("# pick at ")); Serial.print(x, 0); Serial.print(' '); Serial.println(y, 0);
  bool ok = true;
  phase = "APPROACH"; ok = ok && grip(GRIP_OPEN) && swingTo(x, y, Z_HOVER);
  phase = "DESCEND";  ok = ok && lineTo(x, y, Z_GRIP);
  phase = "GRIP";     ok = ok && grip(GRIP_CLOSED);
  phase = "LIFT";     ok = ok && lineTo(x, y, Z_CARRY);
  phase = "TO_BIN";   ok = ok && swingTo(BIN_X, BIN_Y, Z_CARRY);
  phase = "RELEASE";  ok = ok && grip(GRIP_OPEN);
  ok = ok && goHome();
  return ok;
}

uint8_t parseFloats(const char *s, float *out, uint8_t maxCount) {
  uint8_t n = 0;
  char *end;
  while (n < maxCount) {
    float v = strtod(s, &end);
    if (end == s || isnan(v) || isinf(v)) break;  // "nan" and "inf" parse as numbers; never send them to a servo
    out[n++] = v;
    s = end;
  }
  return n;
}

void execute(const char *cmd) {
  float v[4];
  uint8_t n = parseFloats(cmd + 1, v, 4);
  bool ok = false;
  digitalWrite(PIN_LED, HIGH);
  switch (cmd[0]) {
    case 'P': if (n == 2) ok = pick(v[0], v[1]); break;
    case 'G': if (n == 3) { phase = "MOVE"; ok = lineTo(v[0], v[1], v[2]); } break;
    case 'J':
      if (n == 4) {
        for (uint8_t i = 0; i < 4; i++) v[i] = constrain(v[i], 0, 180);
        phase = "MOVE"; tipKnown = false; ok = moveTo(v);
      }
      break;
    case 'O': phase = "MOVE"; ok = grip(GRIP_OPEN); break;
    case 'C': phase = "MOVE"; ok = grip(GRIP_CLOSED); break;
    case 'H': ok = goHome(); break;
    case 'X': ok = true; break;  // nothing was moving
    case 'Z':
    case 'K':
      if (n == 3) {
        float *trim = cmd[0] == 'Z' ? trimOffset : trimGain;
        float limit = cmd[0] == 'Z' ? 20 : 15;
        for (uint8_t i = 0; i < 3; i++) trim[i] = constrain(v[i], -limit, limit);
        saveTrim();
        for (uint8_t i = 0; i < 3; i++) writeServo(i);
      }
      if (n == 3 || n == 0) { printTrim(); ok = true; }
      break;
    case '?':
      Serial.println(F("# P x y | G x y z | J b s e g | O | C | H | X | M ms | Z b s e | K b s e"));
      printTrim();
      ok = true;
      break;
  }
  digitalWrite(PIN_LED, LOW);
  phase = "IDLE";
  telemetry(true);
  Serial.print(ok ? F("OK ") : F("ERR ")); Serial.println(cmd);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  loadTrim();
  for (uint8_t i = 0; i < 4; i++) {
    cur[i] = HOME_POSE[i];
    servos[i].attach(SERVO_PINS[i]);
    writeServo(i);
  }
  Serial.println(F("# Mars arm ready. Try: P 120 20   (or press the green button)"));
  printTrim();
  telemetry(true);
}

void loop() {
  pollSerial();
  int8_t i = nextArrived();
  if (i >= 0) {
    char cmd[LINE_MAX];  // copied, because the slot is free again and a new command may land in it meanwhile
    strcpy(cmd, queue[i].text);
    queue[i].used = false;
    execute(cmd);
  }
  if (millis() - lastTelemetry >= 1000) telemetry(true);  // heartbeat: tells the ground station the link is alive
  if (digitalRead(PIN_BTN) == LOW) {
    float x = 120, y = 20;
#if SIM_POTS
    x = map(analogRead(A0), 0, 1023, 90, 140);
    y = map(analogRead(A1), 0, 1023, -40, 60);
#endif
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "P %d %d", (int)x, (int)y);
    execute(cmd);
  }
}
