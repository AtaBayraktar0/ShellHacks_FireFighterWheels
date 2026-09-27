/* W.A.R.M wheels WN1 bridge -- Arduino Uno, no external libraries.
 * DEMONSTRATION ONLY. Verify your exact motor board before selecting a profile.
 * A software stop removes drive; it is not a mechanical brake or safety relay.
 */
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define PROFILE_DISABLED 0
#define PROFILE_ELEGOO_V4 1
#define PROFILE_L298N_CUSTOM 2

// User-confirmed V4 kit. Keep verification OFF until raised-wheel checks.
// Set HARDWARE_VERIFIED to 1 only after reading docs/hardware.md.
#ifndef MOTOR_PROFILE
#define MOTOR_PROFILE PROFILE_ELEGOO_V4
#endif
#ifndef HARDWARE_VERIFIED
#define HARDWARE_VERIFIED 0
#endif

// For a custom L298N only: replace -1 with your VERIFIED wiring.
#ifndef L298_LEFT_PWM
#define L298_LEFT_PWM -1
#endif
#ifndef L298_LEFT_IN1
#define L298_LEFT_IN1 -1
#endif
#ifndef L298_LEFT_IN2
#define L298_LEFT_IN2 -1
#endif
#ifndef L298_RIGHT_PWM
#define L298_RIGHT_PWM -1
#endif
#ifndef L298_RIGHT_IN1
#define L298_RIGHT_IN1 -1
#endif
#ifndef L298_RIGHT_IN2
#define L298_RIGHT_IN2 -1
#endif

// Adjust only with the wheels raised after testing one side at a time.
#define INVERT_LEFT 0
#define INVERT_RIGHT 0

#if HARDWARE_VERIFIED == 1
#define ACTIVE_MOTOR_PROFILE MOTOR_PROFILE
#else
#define ACTIVE_MOTOR_PROFILE PROFILE_DISABLED
#endif
#if MOTOR_PROFILE < 0 || MOTOR_PROFILE > 2
#error "Unknown motor profile."
#endif

#if ACTIVE_MOTOR_PROFILE == PROFILE_ELEGOO_V4
// ELEGOO official V4.0-New DeviceDriverSet.h: see docs/hardware.md source.
const uint8_t LEFT_PWM = 6, RIGHT_PWM = 5;
const uint8_t LEFT_DIR = 8, RIGHT_DIR = 7, STBY = 3;
#elif ACTIVE_MOTOR_PROFILE == PROFILE_L298N_CUSTOM
const uint8_t LEFT_PWM = L298_LEFT_PWM, RIGHT_PWM = L298_RIGHT_PWM;
const uint8_t LEFT_IN1 = L298_LEFT_IN1, LEFT_IN2 = L298_LEFT_IN2;
const uint8_t RIGHT_IN1 = L298_RIGHT_IN1, RIGHT_IN2 = L298_RIGHT_IN2;
#define VALID_PWM(p) ((p)==3 || (p)==5 || (p)==6 || (p)==9 || (p)==10 || (p)==11)
#define VALID_PIN(p) ((p)>=2 && (p)<=19)
static_assert(VALID_PWM(L298_LEFT_PWM) && VALID_PWM(L298_RIGHT_PWM), "Set verified Uno PWM pins (not pins 0/1).");
static_assert(VALID_PIN(L298_LEFT_IN1) && VALID_PIN(L298_LEFT_IN2) && VALID_PIN(L298_RIGHT_IN1) && VALID_PIN(L298_RIGHT_IN2), "Set verified direction pins.");
static_assert(LEFT_PWM != RIGHT_PWM && LEFT_PWM != LEFT_IN1 && LEFT_PWM != LEFT_IN2 && LEFT_PWM != RIGHT_IN1 && LEFT_PWM != RIGHT_IN2 && RIGHT_PWM != LEFT_IN1 && RIGHT_PWM != LEFT_IN2 && RIGHT_PWM != RIGHT_IN1 && RIGHT_PWM != RIGHT_IN2 && LEFT_IN1 != LEFT_IN2 && LEFT_IN1 != RIGHT_IN1 && LEFT_IN1 != RIGHT_IN2 && LEFT_IN2 != RIGHT_IN1 && LEFT_IN2 != RIGHT_IN2 && RIGHT_IN1 != RIGHT_IN2, "Each motor pin must be distinct.");
#endif

const unsigned long WATCHDOG_MS = 350;
const unsigned long PARTIAL_LINE_MS = 100;
char lineBuffer[48];
uint8_t lineLength = 0;
bool discardLine = false, estopLatched = false, moving = false;
unsigned long lastDriveMs = 0, lastByteMs = 0;

void stopMotors() {
#if ACTIVE_MOTOR_PROFILE == PROFILE_ELEGOO_V4
  analogWrite(LEFT_PWM, 0); analogWrite(RIGHT_PWM, 0);
  digitalWrite(STBY, LOW);
#elif ACTIVE_MOTOR_PROFILE == PROFILE_L298N_CUSTOM
  analogWrite(LEFT_PWM, 0); analogWrite(RIGHT_PWM, 0);
  digitalWrite(LEFT_IN1, LOW); digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW); digitalWrite(RIGHT_IN2, LOW);
#endif
  moving = false;
}

void fault(const __FlashStringHelper *reason) {
  stopMotors();
  estopLatched = true;
  Serial.print(F("ERR ")); Serial.println(reason);
}

void driveMotors(int left, int right) {
  // Remove PWM before changing direction; there is no speed/odometry feedback.
  stopMotors();
  if (INVERT_LEFT) left = -left;
  if (INVERT_RIGHT) right = -right;
#if ACTIVE_MOTOR_PROFILE == PROFILE_ELEGOO_V4
  digitalWrite(LEFT_DIR, left > 0 ? HIGH : LOW);
  digitalWrite(RIGHT_DIR, right > 0 ? HIGH : LOW);
  digitalWrite(STBY, (left || right) ? HIGH : LOW);
  analogWrite(LEFT_PWM, (abs(left) * 255L) / 100);
  analogWrite(RIGHT_PWM, (abs(right) * 255L) / 100);
#elif ACTIVE_MOTOR_PROFILE == PROFILE_L298N_CUSTOM
  digitalWrite(LEFT_IN1, left > 0 ? HIGH : LOW);
  digitalWrite(LEFT_IN2, left < 0 ? HIGH : LOW);
  digitalWrite(RIGHT_IN1, right > 0 ? HIGH : LOW);
  digitalWrite(RIGHT_IN2, right < 0 ? HIGH : LOW);
  analogWrite(LEFT_PWM, (abs(left) * 255L) / 100);
  analogWrite(RIGHT_PWM, (abs(right) * 255L) / 100);
#endif
  moving = left || right;
  lastDriveMs = millis();
}

bool parseInteger(const char *token, long minimum, long maximum, long &value) {
  if (!token || !*token) return false;
  const char *digits = token;
  if (*digits == '-') ++digits;
  if (!*digits) return false;
  for (const char *p = digits; *p; ++p) if (*p < '0' || *p > '9') return false;
  errno = 0;
  char *end;
  value = strtol(token, &end, 10);
  return errno != ERANGE && *end == '\0' && value >= minimum && value <= maximum;
}

void processLine() {
  if (strcmp(lineBuffer, "S") == 0) {
    stopMotors();
    Serial.print(F("STOPPED ")); Serial.println(estopLatched ? 1 : 0); return;
  }
  if (strcmp(lineBuffer, "E") == 0) {
    stopMotors(); estopLatched = true; Serial.println(F("ESTOP 1")); return;
  }
  if (strcmp(lineBuffer, "R") == 0) {
    stopMotors(); estopLatched = false; Serial.println(F("ESTOP 0")); return;
  }
  char *save;
  char *op = strtok_r(lineBuffer, " ", &save);
  char *seqToken = strtok_r(NULL, " ", &save);
  char *leftToken = strtok_r(NULL, " ", &save);
  char *rightToken = strtok_r(NULL, " ", &save);
  char *extra = strtok_r(NULL, " ", &save);
  long sequence, left, right;
  if (!op || strcmp(op, "M") || extra || !parseInteger(seqToken, 0, 2147483647L, sequence) || !parseInteger(leftToken, -100, 100, left) || !parseInteger(rightToken, -100, 100, right)) {
    fault(F("BAD_COMMAND")); return;
  }
#if ACTIVE_MOTOR_PROFILE == PROFILE_DISABLED
  fault(F("MOTORS_DISABLED")); return;
#endif
  if (estopLatched) { stopMotors(); Serial.println(F("ERR ESTOP")); return; }
  driveMotors((int)left, (int)right);
  Serial.print(F("ACK ")); Serial.print(sequence); Serial.print(' ');
  Serial.print(left); Serial.print(' '); Serial.println(right);
}

void setup() {
#if ACTIVE_MOTOR_PROFILE == PROFILE_ELEGOO_V4
  digitalWrite(STBY, LOW); pinMode(STBY, OUTPUT);
  digitalWrite(LEFT_PWM, LOW); digitalWrite(RIGHT_PWM, LOW);
  pinMode(LEFT_PWM, OUTPUT); pinMode(RIGHT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT); pinMode(RIGHT_DIR, OUTPUT);
#elif ACTIVE_MOTOR_PROFILE == PROFILE_L298N_CUSTOM
  digitalWrite(LEFT_PWM, LOW); digitalWrite(RIGHT_PWM, LOW);
  pinMode(LEFT_PWM, OUTPUT); pinMode(RIGHT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT); pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT);
#endif
  stopMotors();
  Serial.begin(115200);
  Serial.println(F("READY WN1"));
#if ACTIVE_MOTOR_PROFILE == PROFILE_ELEGOO_V4
  Serial.println(F("PROFILE V4 VERIFIED"));
#elif ACTIVE_MOTOR_PROFILE == PROFILE_L298N_CUSTOM
  Serial.println(F("PROFILE L298N VERIFIED"));
#elif MOTOR_PROFILE == PROFILE_ELEGOO_V4
  Serial.println(F("PROFILE V4 UNVERIFIED"));
#else
  Serial.println(F("PROFILE DISABLED UNVERIFIED"));
#endif
}

void loop() {
  if (moving && (unsigned long)(millis() - lastDriveMs) >= WATCHDOG_MS) fault(F("WATCHDOG"));
  if (lineLength && (unsigned long)(millis() - lastByteMs) >= PARTIAL_LINE_MS) {
    lineLength = 0; discardLine = true; fault(F("PARTIAL_COMMAND"));
  }
  // Bounded work prevents an incoming byte flood from starving the watchdog.
  for (uint8_t count = 0; count < 32 && Serial.available(); ++count) {
    char c = (char)Serial.read(); lastByteMs = millis();
    if (c == '\n') {
      if (!discardLine) {
        lineBuffer[lineLength] = '\0';
        processLine();
      }
      lineLength = 0; discardLine = false;
    } else if (!discardLine) {
      if (c == '\r') continue; // Accept Serial Monitor CRLF.
      if (c < 32 || c > 126 || lineLength >= sizeof(lineBuffer) - 1) {
        lineLength = 0; discardLine = true; fault(F("BAD_LINE"));
      } else {
        lineBuffer[lineLength++] = c;
      }
    }
  }
}
