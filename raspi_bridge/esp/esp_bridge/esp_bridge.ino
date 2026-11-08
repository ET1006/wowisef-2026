/**
 * ESP32 Bridge — port of arduino_bridge to ESP32
 * --------------------------------------------------
 *  - Motor control, servos, bar-pose logic: same as Arduino version
 *  - Adds: 2x VL53L0X TOF read (up + down) via XSHUT re-addressing
 *  - Adds: 200 ms periodic JSON status exports with prefix "STS:"
 *  - Adds: safety latch on `WARN:<sensor>` lines from the Pi
 *  - Keeps: existing `ACK:<cmd>` lines emitted only on command events
 *
 * Serial protocol on ESP32 -> Pi (line oriented, ASCII):
 *   STS:{...json...}      - one per 200 ms (telemetry, with sensor data)
 *   ACK:<cmd>             - one per command event (e.g. ACK:up_attach)
 *   WARNED:<sensor>       - one per WARN: line received (debug-only echo)
 *
 * Serial protocol on Pi -> ESP32 (CSV line, newline-terminated):
 *   <speed>,<dir>,<cmd>   - e.g. "150,FORWARD,NONE\n"
 *   <dir> in {FORWARD, BACKWARD}
 *   <cmd> in {NONE, up_attach, up_detach, down_attach, down_detach,
 *             both_attach, both_detach, estop}
 *
 *   WARN:<sensor>         - e.g. "WARN:up\n"; immediately stops the
 *                           drive motors and latches.
 *
 *   estop                 - sent as "0,FORWARD,estop". Immediately
 *                           stops the drive motors and latches.
 *
 * ESP32 Pin Map:
 *   - I2C: SDA=21, SCL=22 (default ESP32 I2C pins)
 *   - VL53L0X up XSHUT: GPIO 25
 *   - VL53L0X down XSHUT: GPIO 26
 *   - L298N Motor A: EN=32, IN1=33, IN2=27
 *   - L298N Motor B: EN=14, IN1=12, IN2=13
 *   - LED: GPIO 2 (onboard LED)
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <VL53L0X.h>

// ===================== ESP32 Pin Configuration =====================
// I2C pins
#define I2C_SDA 16
#define I2C_SCL 17

// TOF sensor XSHUT pins
#define TOF_UP_XSHUT 25
#define TOF_DOWN_XSHUT 26

// L298N Motor Driver Pins
#define MOTOR_A_EN 14
#define MOTOR_A_IN1 13
#define MOTOR_A_IN2 12
#define MOTOR_B_EN 11
#define MOTOR_B_IN1 10
#define MOTOR_B_IN2 9

// LED
#define LED_PIN 42

// PWM Configuration for Motors
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

// ===================== TOF Configuration =====================
const int TOF_MEASUREMENT_TIME = 10000;
const bool TOF_ENABLED = false;  // Set to true to enable TOF sensors

VL53L0X tof_up;
VL53L0X tof_down;

// ===================== Sensors state =====================
const uint16_t TOF_SENSOR_DOWN = 0xFFFF;

volatile uint16_t tof_up_mm = TOF_SENSOR_DOWN;
volatile uint16_t tof_down_mm = TOF_SENSOR_DOWN;
volatile bool sensors_ok = false;

bool tof_up_ready = false;
bool tof_down_ready = false;

unsigned long tof_up_next_retry_ms = 0;
unsigned long tof_down_next_retry_ms = 0;
const long TOF_RETRY_AFTER_MS = 5000;

unsigned long last_sensor_read_ms = 0;
const unsigned long SENSOR_READ_TIMEOUT_MS = 50;

const uint8_t TOF_INIT_UP_OK = 0x01;
const uint8_t TOF_INIT_DOWN_OK = 0x02;

// ===================== TOF calibration =====================
struct TofCalib { float slope; float offset; };
const TofCalib TOF_UP_CALIB = { 30.0f / 31.0f, 70.0f - (30.0f / 31.0f) * 87.0f };
const TofCalib TOF_DOWN_CALIB = { 30.0f / 35.0f, 70.0f - (30.0f / 35.0f) * 135.0f };

// Forward declare Direction enum before function prototypes
enum Direction { STOP, FORWARD, BACKWARD };

static uint16_t applyTofCalib(uint16_t raw, const TofCalib& c) {
  float v = c.slope * (float)raw + c.offset;
  if (isnan(v) || isinf(v)) v = 0.0f;
  if (v < 0.0f) v = 0.0f;
  if (v > 65535.0f) v = 65535.0f;
  return (uint16_t)(v + 0.5f);
}

// ===================== Safety latch =====================
bool latched = false;
char latch_reason[16] = "";

// ===================== Status export =====================
unsigned long next_status_ms = 0;
const long STATUS_PERIOD_MS = 200;
unsigned long next_watchdog_ms = 0;
unsigned long next_diag_ms = 0;
const long DIAG_PERIOD_MS = 5000;  // Re-print PCA9685 diagnostics every 5s
char last_cmd[24] = "NONE";
String current_bar_pose = "parallel";
char current_dir[10] = "FORWARD";

// ===================== Servo timing configuration =====================
const unsigned long delay_to_pose = 1500;

// ===================== LED =====================
unsigned long led_flash_timer = 0;
bool led_state = true;

// ===================== Motor state =====================
int current_speed = 0;
Direction current_direction = STOP;

// ===================== Servo Hub =====================
// Two PCA9685 boards:
//   0x40 - Bar servos (pose control)
//   0x60 - Gripper servos (up/down grippers)
Adafruit_PWMServoDriver servo_hub_bar = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver servo_hub_gripper = Adafruit_PWMServoDriver(0x60);

// ----- Diagnostic helpers (kept; these don't depend on the Adafruit lib) -----
// Read MODE1 register (0x00) directly. The SLEEP bit is bit 4 (0x10).
// After the chip is "initialized" the SLEEP bit MUST be 0, otherwise no
// PWM is produced on any channel -- which is exactly the "servos have
// power but don't move" failure mode.
uint8_t readPCA9685Mode1(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x00);
  uint8_t err = Wire.endTransmission();
  if (err != 0) return 0xFF;  // I2C error
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return 0xFF;
  if (!Wire.available()) return 0xFF;
  return Wire.read();
}

// Read PRESCALE register (0xFE). For 50Hz output it MUST be 121 (0x79).
// If it is anything else, every servo pulse will be the wrong width and
// the servos will refuse to move. This is the single most likely cause
// of "firmware says it sent a pulse but the servo stays still".
uint8_t readPCA9685Prescale(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0xFE);
  uint8_t err = Wire.endTransmission();
  if (err != 0) return 0xFF;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return 0xFF;
  if (!Wire.available()) return 0xFF;
  return Wire.read();
}

void printHubWakeupStatus(uint8_t addr, const char* label) {
  uint8_t mode1 = readPCA9685Mode1(addr);
  bool asleep = (mode1 != 0xFF) && ((mode1 & 0x10) != 0);
  Serial.print("[");
  Serial.print(label);
  if (mode1 == 0xFF) {
    Serial.print(" MODE1=I2C_ERR");
  } else {
    Serial.print(" MODE1=0x");
    if (mode1 < 0x10) Serial.print('0');
    Serial.print(mode1, HEX);
    Serial.print(" SLEEP=");
    Serial.print(asleep ? "ON (BAD)" : "OFF (OK)");
  }
  Serial.println("]");
}

void printHubPrescale(uint8_t addr, const char* label) {
  uint8_t prescale = readPCA9685Prescale(addr);
  Serial.print("[");
  Serial.print(label);
  if (prescale == 0xFF) {
    Serial.print(" PRESCALE=I2C_ERR");
  } else {
    Serial.print(" PRESCALE=0x");
    if (prescale < 0x10) Serial.print('0');
    Serial.print(prescale, HEX);
    Serial.print(" (");
    Serial.print(prescale);
    Serial.print(")");
    if (prescale == 121) {
      Serial.print(" =50Hz (OK)");
    } else {
      Serial.print(" !=50Hz (BAD)");
    }
  }
  Serial.println("]");
}

// Bar servo channels on 0x40
const int chServoBarUpL = 15;
const int chServoBarUpR = 14;
const int chServoBarDownL = 13;
const int chServoBarDownR = 12;
const int chServoAxon = 8;

// Gripper servo channels on 0x41
const int chServoUpL = 0;
const int chServoUpR = 1;
const int chServoDownL = 2;
const int chServoDownR = 3;

const float open_position = 180;
const float close_position = 20;
const float angle_change = 40;

// Axon servo positions
const float axon_open = 0;
const float axon_close = 93;

// ===================== Helper Functions =====================
static uint16_t angleToPulse(float angle, float minAngle=0, float maxAngle=180,
                             uint16_t minPulse=1000, uint16_t maxPulse=2000) {
  if (angle < minAngle) angle = minAngle;
  if (angle > maxAngle) angle = maxAngle;
  float t = (angle - minAngle) / (maxAngle - minAngle);
  return (uint16_t)(minPulse + t * (maxPulse - minPulse));
}

static uint16_t angleToPulse2(float angle, float minAngle=0, float maxAngle=180,
                             uint16_t minPulse=500, uint16_t maxPulse=2500) {
  if (angle < minAngle) angle = minAngle;
  if (angle > maxAngle) angle = maxAngle;
  float t = (angle - minAngle) / (maxAngle - minAngle);
  return (uint16_t)(minPulse + t * (maxPulse - minPulse));
}

static uint16_t angleToPulse3(float angle, float minAngle=0, float maxAngle=350,
                             uint16_t minPulse=500, uint16_t maxPulse=2500) {
  if (angle < minAngle) angle = minAngle;
  if (angle > maxAngle) angle = maxAngle;
  float t = (angle - minAngle) / (maxAngle - minAngle);
  return (uint16_t)(minPulse + t * (maxPulse - minPulse));
}

// ===================== Motor Control (ESP32 PWM) =====================
void setupMotorPWM() {
  // Configure PWM channels for motor speed control
  // (ESP32 Arduino core 3.x replaced ledcSetup/ledcAttachPin with ledcAttach)
  ledcAttach(MOTOR_A_EN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_B_EN, PWM_FREQ, PWM_RESOLUTION);
  
  // Configure direction pins
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  
  // Initialize all pins low
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  ledcWrite(MOTOR_A_EN, 0);
  ledcWrite(MOTOR_B_EN, 0);
}

void stopMotors() {
  ledcWrite(MOTOR_A_EN, 0);
  ledcWrite(MOTOR_B_EN, 0);
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  current_speed = 0;
  current_direction = STOP;
}

void setMotors(int speed, Direction dir) {
  if (speed == 0 || dir == STOP) {
    stopMotors();
    return;
  }
  
  current_speed = speed;
  current_direction = dir;
  
  // Set direction for both motors
  if (dir == FORWARD) {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
  } else { // BACKWARD
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
  }
  
  // Set speed (0-255 for 8-bit PWM)
  ledcWrite(MOTOR_A_EN, speed);
  ledcWrite(MOTOR_B_EN, speed);
}

// ===================== Sensor init =====================
uint8_t initTOF() {
  if (!TOF_ENABLED) {
    pinMode(TOF_UP_XSHUT, OUTPUT);
    pinMode(TOF_DOWN_XSHUT, OUTPUT);
    digitalWrite(TOF_UP_XSHUT, LOW);
    digitalWrite(TOF_DOWN_XSHUT, LOW);
    return 0;
  }

  pinMode(TOF_UP_XSHUT, OUTPUT);
  pinMode(TOF_DOWN_XSHUT, OUTPUT);
  digitalWrite(TOF_DOWN_XSHUT, LOW);
  digitalWrite(TOF_UP_XSHUT, LOW);
  delay(50);
  digitalWrite(TOF_UP_XSHUT, HIGH);
  delay(50);
  
  uint8_t r = 0;
  if (tof_up.init()) {
    tof_up.setAddress(0x30);
    tof_up.startContinuous();
    r |= TOF_INIT_UP_OK;
  }
  
  digitalWrite(TOF_DOWN_XSHUT, HIGH);
  delay(50);
  if (tof_down.init()) {
    tof_down.setAddress(0x31);
    tof_down.startContinuous();
    r |= TOF_INIT_DOWN_OK;
  }
  return r;
}

bool reinitOneSensor(bool which_up) {
  if (!TOF_ENABLED) {
    return false;
  }
  if (which_up) {
    digitalWrite(TOF_UP_XSHUT, LOW);
    delay(20);
    digitalWrite(TOF_UP_XSHUT, HIGH);
    delay(50);
    if (!tof_up.init()) return false;
    tof_up.setAddress(0x30);
    tof_up.startContinuous();
    return true;
  } else {
    digitalWrite(TOF_DOWN_XSHUT, LOW);
    delay(20);
    digitalWrite(TOF_DOWN_XSHUT, HIGH);
    delay(50);
    if (!tof_down.init()) return false;
    tof_down.setAddress(0x31);
    tof_down.startContinuous();
    return true;
  }
}

void readSensors() {
  if (!TOF_ENABLED) {
    tof_up_mm = TOF_SENSOR_DOWN;
    tof_down_mm = TOF_SENSOR_DOWN;
    return;
  }

  unsigned long now = millis();
  
  if ((long)(now - last_sensor_read_ms) < SENSOR_READ_TIMEOUT_MS) {
    return;
  }
  last_sensor_read_ms = now;

  uint16_t raw_up = tof_up_ready ? tof_up.readRangeContinuousMillimeters() : TOF_SENSOR_DOWN;
  uint16_t raw_down = tof_down_ready ? tof_down.readRangeContinuousMillimeters() : TOF_SENSOR_DOWN;

  if (raw_up == TOF_SENSOR_DOWN || raw_up == 0 || tof_up.timeoutOccurred()) {
    tof_up_mm = TOF_SENSOR_DOWN;
  } else {
    tof_up_mm = applyTofCalib(raw_up, TOF_UP_CALIB);
  }

  if (raw_down == TOF_SENSOR_DOWN || raw_down == 0 || tof_down.timeoutOccurred()) {
    tof_down_mm = TOF_SENSOR_DOWN;
  } else {
    tof_down_mm = applyTofCalib(raw_down, TOF_DOWN_CALIB);
  }
}

void ensureSensorsAlive() {
  if (!TOF_ENABLED) {
    return;
  }

  unsigned long now = millis();

  if (tof_up_mm == TOF_SENSOR_DOWN && (long)(now - tof_up_next_retry_ms) >= 0) {
    if (reinitOneSensor(true)) {
      tof_up_ready = true;
      tof_up_mm = TOF_SENSOR_DOWN;
      tof_up_next_retry_ms = now + TOF_RETRY_AFTER_MS;
      Serial.println("TOF: up re-init OK");
    } else {
      tof_up_next_retry_ms = now + TOF_RETRY_AFTER_MS;
      Serial.println("TOF: up re-init FAIL");
    }
  } else if (tof_up_mm != TOF_SENSOR_DOWN) {
    tof_up_next_retry_ms = now + TOF_RETRY_AFTER_MS;
  }

  if (tof_down_mm == TOF_SENSOR_DOWN && (long)(now - tof_down_next_retry_ms) >= 0) {
    if (reinitOneSensor(false)) {
      tof_down_ready = true;
      tof_down_mm = TOF_SENSOR_DOWN;
      tof_down_next_retry_ms = now + TOF_RETRY_AFTER_MS;
      Serial.println("TOF: down re-init OK");
    } else {
      tof_down_next_retry_ms = now + TOF_RETRY_AFTER_MS;
      Serial.println("TOF: down re-init FAIL");
    }
  } else if (tof_down_mm != TOF_SENSOR_DOWN) {
    tof_down_next_retry_ms = now + TOF_RETRY_AFTER_MS;
  }
}

// ===================== JSON Status Output =====================
void emitStatusLine() {
  uint16_t safe_tof_up = tof_up_mm;
  uint16_t safe_tof_down = tof_down_mm;
  
  const char* dir_s = (current_direction == FORWARD) ? "FORWARD" :
                      (current_direction == BACKWARD) ? "BACKWARD" : "STOP";

  Serial.print("STS:{"
                 "\"speed\":");
  Serial.print(current_speed);
  Serial.print(",\"dir\":\"");
  Serial.print(dir_s);
  Serial.print("\",\"pose\":\"");
  Serial.print(current_bar_pose);
  Serial.print("\",\"ack\":\"");
  Serial.print(last_cmd);
  Serial.print("\",\"latched\":");
  Serial.print(latched ? "true" : "false");
  if (latched) {
    Serial.print(",\"latch_reason\":\"");
    Serial.print(latch_reason);
    Serial.print("\"");
  }
  Serial.print(",\"tof\":{"
                 "\"enabled\":");
  Serial.print(TOF_ENABLED ? "true" : "false");
  Serial.print(",\"up\":");
  Serial.print(safe_tof_up);
  Serial.print(",\"down\":");
  Serial.print(safe_tof_down);
  Serial.print("}}");
  Serial.println();
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  // 100ms timeout is the sweet spot: long enough that an OS-side serial
  // chunk (where pyserial's write() gets split into bursts by the USB CDC
  // driver) never falsely terminates a line in the middle, short enough
  // that the main loop stays responsive. The previous 10ms value was
  // tripping "ERR:malformed CSV" on otherwise well-formed commands.
  Serial.setTimeout(100);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Setup motor PWM
  setupMotorPWM();
  stopMotors();

  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  // 100kHz is the default and is the safest choice on ESP32: the chip's
  // internal pull-ups are ~45kΩ, which is borderline at 400kHz and can
  // cause writes to silently NACK on clones. Servo writes here are
  // bursty but infrequent (one per pose change), so 100kHz is fine.
  Wire.setClock(100000);

  // One-time I2C scan so we can confirm both hub addresses are on the bus
  // before we try to drive them.
  Serial.print("I2C scan:");
  uint8_t found_count = 0;
  for (uint8_t addr = 0x03; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(" 0x");
      if (addr < 0x10) Serial.print('0');
      Serial.print(addr, HEX);
      found_count++;
    }
  }
  Serial.print(" (");
  Serial.print(found_count);
  Serial.println(" device(s))");

  // Initialize servo hubs via the Adafruit library.
  //
  // Sequence matters: Adafruit_PWMServoDriver::setPWMFreq() leaves the
  // chip with the SLEEP bit set in MODE1. On Arduino (5V) the chip wakes
  // itself up implicitly on the next write, but on ESP32 (3.3V) it stays
  // asleep and no PWM is produced on ANY channel until you call wakeup().
  // Without that call the only thing that appears to "work" is whichever
  // channel happens to be looked at when the chip finally does wake.
  servo_hub_bar.begin();
  servo_hub_bar.reset();
  servo_hub_bar.setPWMFreq(50);
  servo_hub_bar.wakeup();
  delay(5);
  printHubWakeupStatus(0x40, "bar");
  printHubPrescale(0x40, "bar");

  servo_hub_gripper.begin();
  servo_hub_gripper.reset();
  servo_hub_gripper.setPWMFreq(50);
  servo_hub_gripper.wakeup();
  delay(5);
  printHubWakeupStatus(0x60, "gripper");
  printHubPrescale(0x60, "gripper");

  Wire.begin(I2C_SDA, I2C_SCL);

  // TOF sensor initialization
  pinMode(TOF_UP_XSHUT, OUTPUT);
  pinMode(TOF_DOWN_XSHUT, OUTPUT);
  digitalWrite(TOF_UP_XSHUT, LOW);
  digitalWrite(TOF_DOWN_XSHUT, LOW);

  uint8_t tof_r = initTOF();
  tof_up_ready = (tof_r & TOF_INIT_UP_OK) != 0;
  tof_down_ready = (tof_r & TOF_INIT_DOWN_OK) != 0;
  sensors_ok = (tof_r != 0);
  
  unsigned long _t0 = millis();
  tof_up_next_retry_ms = _t0 + TOF_RETRY_AFTER_MS;
  tof_down_next_retry_ms = _t0 + TOF_RETRY_AFTER_MS;

  Serial.print("INIT: tof up=");
  Serial.print(tof_up_ready ? "OK" : "FAIL");
  Serial.print(" down=");
  Serial.println(tof_down_ready ? "OK" : "FAIL");

  if (tof_up_ready) {
    tof_up.setMeasurementTimingBudget(TOF_MEASUREMENT_TIME);
  }
  if (tof_down_ready) {
    tof_down.setMeasurementTimingBudget(TOF_MEASUREMENT_TIME);
  }

  // Initialize servos to default positions
  setBarPosition("parallel");
  setAxonPosition(axon_close);
  
  // Initialize all gripper servos to closed position at startup
  setServoAngle(chServoUpL, chServoUpR, close_position);
  setServoAngle(chServoDownL, chServoDownR, close_position);
  // Re-print the diagnostic state right at the end of setup() so that
  // even if you open the serial monitor AFTER the chip boots, the next
  // time you look you'll still see the current hub state.
  Serial.println("==================================================");
  Serial.println("BOOT COMPLETE (still in setup)");
  printHubWakeupStatus(0x40, "bar");
  printHubPrescale(0x40, "bar");
  printHubWakeupStatus(0x60, "gripper");
  printHubPrescale(0x60, "gripper");
  Serial.println("(This block re-prints every 5s in loop().)");
  Serial.println("==================================================");

  Serial.println("ESP32 Bridge Ready");
}

// ===================== Main loop =====================
void loop() {
  unsigned long now = millis();
  
  // 1. Inbound commands / warnings
  if (Serial.available() > 0) {
    String payload = Serial.readStringUntil('\n');
    payload.trim();
    
    if (payload.length() > 128) {
      Serial.println("ERR:line too long");
      return;
    }
    
    if (payload.length() > 0) {
      if (payload.startsWith("WARN:")) {
        parseAndExecuteWarning(payload);
      } else {
        parseAndExecutePayload(payload);
      }
    }
  }

  // 2. Sensor watchdog
  if ((long)(now - next_watchdog_ms) >= 0) {
    next_watchdog_ms = now + 1000;
    ensureSensorsAlive();
  }

  // 2.5 Periodic re-print of PCA9685 diagnostics (5 s). This is here so
  // that even if you open the serial monitor AFTER boot, you'll still
  // see the current state of each hub within a few seconds. It's the
  // single most useful log line for figuring out "why didn't the servos
  // move".
  if ((long)(now - next_diag_ms) >= 0) {
    next_diag_ms = now + DIAG_PERIOD_MS;
    Serial.println("[diag] (re-print every 5s)");
    printHubWakeupStatus(0x40, "bar");
    printHubPrescale(0x40, "bar");
    printHubWakeupStatus(0x60, "gripper");
    printHubPrescale(0x60, "gripper");
  }

  // 3. Periodic status export (200 ms)
  if ((long)(now - next_status_ms) >= 0) {    next_status_ms = now + STATUS_PERIOD_MS;
    readSensors();
    emitStatusLine();
  }

  // 4. LED flash when drive is active
  if (current_speed > 0) {
    if (now - led_flash_timer >= 150) {
      led_state = !led_state;
      digitalWrite(LED_PIN, led_state ? HIGH : LOW);
      led_flash_timer = now;
    }
  } else {
    digitalWrite(LED_PIN, LOW);
    led_state = true;
    led_flash_timer = now;
  }
}

// ===================== Command parsing =====================
void parseAndExecutePayload(String payload) {
  // Protocol is line-oriented CSV. Two wire formats are accepted:
  //   3-field:  <speed>,<dir>,<cmd>
  //   4-field:  <speed>,<dir>,<cmd>,<altitude_mode>   (gamepad_to_arduino_direct.py)
  // For the 4-field variant, everything between the 2nd and 3rd comma is the
  // command and we silently drop altitude_mode. If we used substring(2nd,)
  // we'd get "up_attach,manual" which would silently miss every recognized
  // command and fall into the unknown-command ACK path -- so motors would
  // still drive but servos never move.
  int firstComma = payload.indexOf(',');
  int secondComma = (firstComma == -1) ? -1 : payload.indexOf(',', firstComma + 1);
  int thirdComma  = (secondComma == -1) ? -1 : payload.indexOf(',', secondComma + 1);

  if (firstComma == -1 || secondComma == -1 || firstComma == 0) {
    Serial.println("ERR:malformed CSV");
    return;
  }

  String speedStr = payload.substring(0, firstComma);
  String directionStr = payload.substring(firstComma + 1, secondComma);
  String commandStr = (thirdComma == -1)
                        ? payload.substring(secondComma + 1)
                        : payload.substring(secondComma + 1, thirdComma);
  commandStr.trim();

  if (speedStr.length() == 0 || directionStr.length() > 9 || commandStr.length() > 20) {
    Serial.println("ERR:field length");
    return;
  }

  if (latched) {
    if (commandStr == "NONE") {
      return;
    }
    latched = false;
    latch_reason[0] = '\0';
  }

  int speedVal = speedStr.toInt();
  if (speedVal == 0 && speedStr != "0") {
    Serial.println("ERR:non-numeric speed");
    return;
  }

  if (directionStr != "FORWARD" && directionStr != "BACKWARD") {
    Serial.println("ERR:invalid direction");
    return;
  }

  strncpy(current_dir, directionStr.c_str(), sizeof(current_dir) - 1);
  current_dir[sizeof(current_dir) - 1] = '\0';

  setDriveMotors(speedVal, directionStr);

  if (commandStr != "NONE") {
    executeCommand(commandStr);
  }
}

void parseAndExecuteWarning(String payload) {
  String sensor = payload.substring(5);
  sensor.trim();
  if (sensor != "up" && sensor != "down") {
    return;
  }
  if (!TOF_ENABLED) {
    Serial.print("ERR:tof disabled:");
    Serial.println(sensor);
    return;
  }
  stopMotors();
  digitalWrite(LED_PIN, LOW);
  latched = true;
  strncpy(latch_reason, sensor.c_str(), sizeof(latch_reason) - 1);
  latch_reason[sizeof(latch_reason) - 1] = '\0';
  Serial.print("WARNED:");
  Serial.println(sensor);
}

void setDriveMotors(int speed, String direction) {
  if (speed == 0) {
    stopMotors();
    return;
  }

  Direction dir;
  if (direction == "FORWARD") {
    dir = FORWARD;
  } else if (direction == "BACKWARD") {
    dir = BACKWARD;
  } else {
    Serial.println("ERR:invalid direction in setDriveMotors");
    stopMotors();
    return;
  }

  setMotors(speed, dir);
  digitalWrite(LED_PIN, HIGH);
}

void executeCommand(String cmd) {
  if (cmd == "down_detach") {
    down_detach();
  }
  else if (cmd == "down_attach") {
    down_attach();
  }
  else if (cmd == "up_attach") {
    up_attach();
  }
  else if (cmd == "up_detach") {
    up_detach();
  }
  else if (cmd == "both_attach") {
    both_attach();
  }
  else if (cmd == "both_detach") {
    both_detach();
  }
  else if (cmd == "axon_open") {
    setAxonPosition(axon_open);
  }
  else if (cmd == "axon_close") {
    setAxonPosition(axon_close);
  }
  else if (cmd == "estop") {
    stopMotors();
    digitalWrite(LED_PIN, LOW);
    if (!latched) {
      latched = true;
      strncpy(latch_reason, "estop", sizeof(latch_reason) - 1);
      latch_reason[sizeof(latch_reason) - 1] = '\0';
    }
  }
  else {
    Serial.print("ACK:");
    Serial.println(cmd);
    return;
  }
  
  strncpy(last_cmd, cmd.c_str(), sizeof(last_cmd) - 1);
  last_cmd[sizeof(last_cmd) - 1] = '\0';
  Serial.print("ACK:");
  Serial.println(cmd);
}

void setBarPosition(String pose) {
  Serial.print("[bar] pose=");
  Serial.print(pose);
  if (pose == "parallel") {
    servo_hub_bar.writeMicroseconds(chServoBarUpL, angleToPulse2(90 + 10));
    servo_hub_bar.writeMicroseconds(chServoBarUpR, angleToPulse2(90 - 10));
    servo_hub_bar.writeMicroseconds(chServoBarDownL, angleToPulse2(90 - 3));
    servo_hub_bar.writeMicroseconds(chServoBarDownR, angleToPulse2(90 + 3));
    Serial.print(" UL=90 UR=90 DL=90 DR=90 us=");
    Serial.print(angleToPulse2(90));
  }
  else if (pose == "up_detach") {
    uint16_t dL = angleToPulse2(90 + angle_change);
    uint16_t dR = angleToPulse2(180 - (90 + angle_change));
    servo_hub_bar.writeMicroseconds(chServoBarDownL, dL);
    servo_hub_bar.writeMicroseconds(chServoBarDownR, dR);
    servo_hub_bar.writeMicroseconds(chServoBarUpL, angleToPulse2(90));
    servo_hub_bar.writeMicroseconds(chServoBarUpR, angleToPulse2(90));
    Serial.print(" UL=90 UR=90 DL=");
    Serial.print(dL);
    Serial.print(" DR=");
    Serial.print(dR);
  }
  else if (pose == "down_detach") {
    uint16_t uL = angleToPulse2(90 - angle_change);
    uint16_t uR = angleToPulse2(180 - (90 - angle_change));
    servo_hub_bar.writeMicroseconds(chServoBarUpL, uL);
    servo_hub_bar.writeMicroseconds(chServoBarUpR, uR);
    servo_hub_bar.writeMicroseconds(chServoBarDownL, angleToPulse2(90));
    servo_hub_bar.writeMicroseconds(chServoBarDownR, angleToPulse2(90));
    Serial.print(" UL=");
    Serial.print(uL);
    Serial.print(" UR=");
    Serial.print(uR);
    Serial.print(" DL=90 DR=90");
  }
  Serial.println();
  current_bar_pose = pose;
}

void setServoAngle(int channelL, int channelR, float angle) {
  uint16_t pulseL = angleToPulse(angle);
  uint16_t pulseR = angleToPulse(180 - angle - 5);
  servo_hub_gripper.writeMicroseconds(channelL, pulseL);
  servo_hub_gripper.writeMicroseconds(channelR, pulseR);
  Serial.print("[grip] ch");
  Serial.print(channelL);
  Serial.print("=");
  Serial.print(pulseL);
  Serial.print("us ch");
  Serial.print(channelR);
  Serial.print("=");
  Serial.print(pulseR);
  Serial.print("us angle=");
  Serial.println(angle);
}

void up_detach() {
  setServoAngle(chServoUpL, chServoUpR, open_position);
  delay(delay_to_pose);
  setBarPosition("up_detach");
}

void down_detach() {
  setServoAngle(chServoDownL, chServoDownR, open_position);
  delay(delay_to_pose);
  setBarPosition("down_detach");
}

void up_attach() {
  setBarPosition("parallel");
  delay(delay_to_pose);
  setServoAngle(chServoUpL, chServoUpR, close_position);
  setServoAngle(chServoDownL, chServoDownR, close_position);
}

void down_attach() {
  setBarPosition("parallel");
  delay(delay_to_pose);
  setServoAngle(chServoUpL, chServoUpR, close_position);
  setServoAngle(chServoDownL, chServoDownR, close_position);
}

void both_attach() {
  setBarPosition("parallel");
  delay(delay_to_pose);
  setServoAngle(chServoUpL, chServoUpR, close_position);
  setServoAngle(chServoDownL, chServoDownR, close_position);
}

void both_detach() {
  setBarPosition("parallel");
  delay(delay_to_pose);
  setServoAngle(chServoUpL, chServoUpR, open_position);
  setServoAngle(chServoDownL, chServoDownR, open_position);
}

void setAxonPosition(float angle) {
  uint16_t pulse = angleToPulse3(angle);
  servo_hub_bar.writeMicroseconds(chServoAxon, pulse);
  Serial.print("[axon] ch");
  Serial.print(chServoAxon);
  Serial.print("=");
  Serial.print(pulse);
  Serial.print("us angle=");
  Serial.println(angle);
}
