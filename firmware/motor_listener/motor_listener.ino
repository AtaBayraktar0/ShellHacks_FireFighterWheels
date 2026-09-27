#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <avr/wdt.h>

// Check these pins against your Elegoo shield revision.
const byte RIGHT_PWM = 5, LEFT_PWM = 6;
const byte RIGHT_DIR = 7, LEFT_DIR = 8, STBY = 3;
const bool LEFT_FORWARD_HIGH = true, RIGHT_FORWARD_HIGH = true;
char buffer[48];
byte used = 0;
bool dropping = false, moving = false;
unsigned long deadline = 0, lastByte = 0;

void stopMotors() {
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
  digitalWrite(STBY, LOW);
  moving = false;
}

bool number(const char *text, long &value) {
  if (!text || !*text) return false;
  for (const char *p = text; *p; ++p) if (*p < '0' || *p > '9') return false;
  if (strlen(text) > 3) return false;
  value = strtol(text, NULL, 10);
  return true;
}

void command() {
  stopMotors();
  char *name = strtok(buffer, " ");
  char *speedText = strtok(NULL, " ");
  char *timeText = strtok(NULL, " ");
  char *extra = strtok(NULL, " ");
  if (name && strcmp(name, "STOP") == 0 && !speedText) {
    Serial.println(F("OK"));
    return;
  }
  long speed = 0, duration = 0;
  bool known = name && (!strcmp(name, "FWD") || !strcmp(name, "BACK") ||
                        !strcmp(name, "LEFT") || !strcmp(name, "RIGHT"));
  if (!known || extra || !number(speedText, speed) || !number(timeText, duration) ||
      speed < 1 || speed > 120 || duration < 1 || duration > 200) {
    Serial.println(F("ERR"));
    return;
  }
  bool leftForward = strcmp(name, "BACK") && strcmp(name, "LEFT");
  bool rightForward = strcmp(name, "BACK") && strcmp(name, "RIGHT");
  digitalWrite(LEFT_DIR, leftForward == LEFT_FORWARD_HIGH ? HIGH : LOW);
  digitalWrite(RIGHT_DIR, rightForward == RIGHT_FORWARD_HIGH ? HIGH : LOW);
  digitalWrite(STBY, HIGH);
  analogWrite(LEFT_PWM, speed);
  analogWrite(RIGHT_PWM, speed);
  moving = true;
  deadline = millis() + duration;
  Serial.println(F("OK"));
}

void setup() {
  MCUSR = 0;
  wdt_disable();
  pinMode(LEFT_PWM, OUTPUT); pinMode(RIGHT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT); pinMode(RIGHT_DIR, OUTPUT); pinMode(STBY, OUTPUT);
  stopMotors();
  Serial.begin(115200);
  wdt_enable(WDTO_500MS);
}

void loop() {
  wdt_reset();
  // Stop even if the Pi freezes or its USB cable comes out.
  if (moving && (long)(millis() - deadline) >= 0) stopMotors();
  if ((used || dropping) && millis() - lastByte > 100) {
    used = 0; dropping = false; stopMotors();
  }
  if (!Serial.available()) return;
  char c = Serial.read();
  lastByte = millis();
  if (c == '\r') return;
  if (c == '\n') {
    if (dropping) Serial.println(F("ERR"));
    else { buffer[used] = '\0'; command(); }
    used = 0; dropping = false;
  } else if ((unsigned char)c < 32 || (unsigned char)c > 126 || used >= sizeof(buffer)-1) {
    stopMotors(); dropping = true;
  } else if (!dropping) buffer[used++] = c;
}
