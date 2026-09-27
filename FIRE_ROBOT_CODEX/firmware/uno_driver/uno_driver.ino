// ELEGOO Smart Robot Car V4.0 (SmartCar-Shield-V1.1, TB6612FNG)
// Serial motor driver: the Raspberry Pi sends line commands over USB,
// the Uno drives the motors. See ../../README.md for the protocol.

#include <Servo.h>  // uses Timer1: disables PWM on pins 9/10 (unused here)
#include <Wire.h>   // MPU6050 on A4/A5

// --- Pins (verified against the SmartCar-Shield-V1.1 schematic) ---
const int PWMA = 5;   // right motors (M2, M3) speed
const int PWMB = 6;   // left motors (M1, M4) speed
const int AIN1 = 7;   // right direction, HIGH = forward (AIN2 is made by the U3 inverter)
const int BIN1 = 8;   // left direction, HIGH = forward (BIN2 is made by the U3 inverter)
const int STBY = 3;   // must be HIGH or the TB6612 ignores everything
const int TRIG = 13;  // HC-SR04 trigger: we pulse this to send a ping
const int ECHO = 12;  // HC-SR04 echo: goes HIGH for as long as the sound took to return
const int PAN = 11;   // Servo 1: camera pan

// --- Tuning ---
const long BAUD = 115200;                  // serial speed; must match UnoLink on the Pi
const int MIN_PWM = 30;                    // below this the motors stall
const unsigned long WATCHDOG_MS = 500;     // stop if the Pi goes quiet
const unsigned long SONAR_PERIOD_MS = 60;  // HC-SR04 needs ~60 ms between pings
const unsigned long ECHO_TIMEOUT_US = 25000;  // ~4 m; keeps loop() responsive
const int STOP_CM = 15;                    // safety stop distance
const uint8_t MPU = 0x68;                  // GY-521 / MPU6050 I2C address
const float GYRO_LSB_PER_DPS = 131.0;      // +-250 dps range: raw 131 = 1 degree/second
const float GYRO_DEADBAND_DPS = 1.0;       // ignore tiny rates (bias leftovers)
const unsigned long GYRO_PERIOD_US = 5000; // read the gyro every 5 ms (200 Hz)

// --- State ---
int curLeft = 0, curRight = 0;       // PWM currently applied to each side (signed)
unsigned long lastCmdMs = 0;         // when the last valid command arrived (for the watchdog)
unsigned long lastSonarMs = 0;       // when the ultrasonic last fired
long lastCm = -1;      // -1 = no echo (nothing in range)
uint8_t closeCount = 0;              // how many readings in a row were under STOP_CM
bool blocked = false;  // two consecutive readings under STOP_CM

Servo pan;                           // the camera pan servo

bool imuOk = false;    // false if the MPU6050 didn't answer at startup
float yawDeg = 0;      // integrated gyro Z, CCW positive if the chip is face up
float gyroBias = 0;    // gyro reading when still; subtracted from every sample
unsigned long lastGyroUs = 0;  // time of the previous gyro sample, microseconds

char buf[32];          // the command line being received
uint8_t len = 0;       // how many characters are in buf so far
bool overflow = false; // true if the current line was too long for buf

// Clamp a speed to -255..255, and treat anything too slow to move the motors as 0.
int shape(int v) {
  v = constrain(v, -255, 255);            // PWM can't go beyond 255
  return abs(v) < MIN_PWM ? 0 : v;        // tiny values would just hum, so stop instead
}

// Drive one side: the sign picks the direction, the size sets the speed.
void setSide(int dirPin, int pwmPin, int v) {
  digitalWrite(dirPin, v >= 0 ? HIGH : LOW);  // HIGH = forward, LOW = reverse
  analogWrite(pwmPin, abs(v));                // speed 0..255
}

// Set both sides at once (tank drive) and remember what we set.
void drive(int left, int right) {
  curLeft = shape(left);
  curRight = shape(right);
  setSide(BIN1, PWMB, curLeft);   // B channel = left motors
  setSide(AIN1, PWMA, curRight);  // A channel = right motors
}

void stopCar() { drive(0, 0); }

// True if the car is going forward overall. A spin in place sums to 0, so it doesn't count.
bool movingForward() { return curLeft + curRight > 0; }

// Fire one ultrasonic ping and return the distance in cm, or -1 if no echo came back.
long readCm() {
  digitalWrite(TRIG, LOW);          // make sure the trigger starts low
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);         // a 10 us pulse starts a ping
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long us = pulseIn(ECHO, HIGH, ECHO_TIMEOUT_US);  // round-trip time of the sound
  return us == 0 ? -1 : (long)(us / 58);  // 58 us per cm (there and back); 0 = timed out
}

// Called every SONAR_PERIOD_MS: take a reading and stop the car if something is too close.
void updateSonar() {
  lastCm = readCm();
  if (lastCm > 0 && lastCm < STOP_CM) {
    if (closeCount < 2) closeCount++;  // count close readings in a row (capped at 2)
  } else {
    closeCount = 0;                    // one clear reading resets the count
  }
  blocked = closeCount >= 2;  // debounce single noisy readings

  if (blocked && movingForward()) {    // only forward motion is dangerous here
    stopCar();
    Serial.print(F("E STOP_OBSTACLE "));  // tell the Pi why we stopped
    Serial.println(lastCm);
  }
}

// Drive request with the safety check applied. Returns false if refused.
bool safeDrive(int left, int right) {
  if (blocked && shape(left) + shape(right) > 0) {  // forward into an obstacle: refuse
    stopCar();
    Serial.print(F("ERR BLOCKED "));
    Serial.println(lastCm);
    return false;
  }
  drive(left, right);        // reverse and spin are always allowed so the Pi can back out
  Serial.println(F("OK"));
  return true;
}

// --- MPU6050 gyro heading ---

// Write one byte to one MPU6050 register. Returns true if the chip acknowledged.
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);                     // which register
  Wire.write(val);                     // what to put in it
  return Wire.endTransmission() == 0;  // 0 = success
}

// Read the gyro's Z axis (rotation around vertical = turning left/right).
bool readGyroZ(int16_t &out) {
  Wire.beginTransmission(MPU);
  Wire.write(0x47);  // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return false;         // false = keep the bus for the read
  if (Wire.requestFrom(MPU, (uint8_t)2) != 2) return false;   // ask for the 2 bytes of Z
  uint8_t hi = Wire.read();            // high byte first
  uint8_t lo = Wire.read();
  out = (int16_t)((hi << 8) | lo);     // combine into a signed 16-bit value
  return true;
}

// The car must be still for ~1 s here: it measures the gyro's zero offset.
void imuSetup() {
  Wire.begin();
  Wire.setWireTimeout(3000, true);  // don't hang if the IMU is missing
  imuOk = mpuWrite(0x6B, 0x00)      // wake up
       && mpuWrite(0x1A, 0x03)      // 44 Hz low-pass filter
       && mpuWrite(0x1B, 0x00);     // +-250 dps
  if (!imuOk) return;               // no IMU: 'H' will answer ERR NO_IMU
  delay(100);                       // let the gyro settle after waking
  long sum = 0;
  int n = 0;
  for (int i = 0; i < 200; i++) {   // ~1 s of samples while the car sits still
    int16_t z;
    if (readGyroZ(z)) { sum += z; n++; }
    delay(4);
  }
  if (n < 100) { imuOk = false; return; }  // too many failed reads: don't trust it
  gyroBias = (float)sum / n;        // average reading at rest = the offset to subtract
  lastGyroUs = micros();
}

// Called every loop: add (rotation rate x time since last sample) to the heading.
void updateYaw() {
  unsigned long now = micros();
  if (!imuOk || now - lastGyroUs < GYRO_PERIOD_US) return;  // not time yet
  int16_t z;
  if (!readGyroZ(z)) return;
  float dt = (now - lastGyroUs) * 1e-6;             // seconds since the last sample
  lastGyroUs = now;
  float rate = (z - gyroBias) / GYRO_LSB_PER_DPS;   // degrees per second
  if (fabs(rate) > GYRO_DEADBAND_DPS) yawDeg += rate * dt;  // ignore noise when still
}

// Run one complete command line, e.g. "M 120 -120".
void handleLine(char *line) {
  char cmd = toupper(line[0]);                // first letter is the command
  int a = 0, b = 0;
  int n = sscanf(line + 1, "%d %d", &a, &b);  // up to two numbers after it; n = how many

  switch (cmd) {
    case 'M':                                 // M <left> <right>: tank drive
      if (n != 2) { Serial.println(F("ERR ARGS")); return; }
      safeDrive(a, b);
      break;
    case 'F':
    case 'B':
    case 'L':
    case 'R': {                               // one speed, direction from the letter
      if (n < 1) { Serial.println(F("ERR ARGS")); return; }
      int s = constrain(a, 0, 255);
      if (cmd == 'F') safeDrive(s, s);        // forward
      else if (cmd == 'B') safeDrive(-s, -s); // back
      else if (cmd == 'L') safeDrive(-s, s);  // spin left: left side back, right forward
      else safeDrive(s, -s);                  // spin right
      break;
    }
    case 'S':                                 // stop
      stopCar();
      Serial.println(F("OK"));
      break;
    case 'D':                                 // latest ultrasonic distance
      Serial.print(F("D "));
      Serial.println(lastCm);
      break;
    case 'P':                                 // ping / keepalive
      Serial.println(F("OK"));
      break;
    case 'H':                                 // gyro heading
      if (!imuOk) { Serial.println(F("ERR NO_IMU")); return; }
      Serial.print(F("H "));
      Serial.println((long)(yawDeg * 100));  // centidegrees
      break;
    case 'V':                                 // camera pan angle
      if (n < 1) { Serial.println(F("ERR ARGS")); return; }
      pan.write(constrain(a, 0, 180));
      Serial.println(F("OK"));
      break;
    default:
      Serial.println(F("ERR UNKNOWN"));
      return;                                 // invalid commands don't feed the watchdog
  }
  lastCmdMs = millis();                       // any valid command proves the Pi is alive
}

// Collect incoming characters into buf without ever waiting; run each line when '\n' arrives.
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;                  // ignore Windows-style line endings
    if (c == '\n') {                          // end of a command
      if (overflow) Serial.println(F("ERR TOO_LONG"));
      else if (len > 0) { buf[len] = '\0'; handleLine(buf); }  // terminate and run it
      len = 0;                                // start a fresh line
      overflow = false;
    } else if (len < sizeof(buf) - 1) {       // leave room for the '\0'
      buf[len++] = c;
    } else {
      overflow = true;                        // too long: drop it and report at '\n'
    }
  }
}

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  stopCar();                 // make sure the motors are off before enabling the driver
  digitalWrite(STBY, HIGH);  // wake the TB6612
  pan.attach(PAN);
  pan.write(90);             // camera straight ahead
  imuSetup();                // takes ~1 s: keep the car still

  Serial.begin(BAUD);
  Serial.println(F("READY"));  // the Pi waits for this before sending anything
}

void loop() {
  readSerial();              // handle any commands from the Pi
  updateYaw();               // keep the heading up to date

  unsigned long now = millis();
  if ((curLeft || curRight) && now - lastCmdMs > WATCHDOG_MS) {  // moving, but Pi went quiet
    stopCar();
    Serial.println(F("E WATCHDOG"));
  }
  if (now - lastSonarMs >= SONAR_PERIOD_MS) {  // time for the next ultrasonic ping
    lastSonarMs = now;
    updateSonar();
  }
}
