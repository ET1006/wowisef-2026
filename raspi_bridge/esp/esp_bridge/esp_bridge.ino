#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define I2C_SDA 16
#define I2C_SCL 17
#define LED_PIN 42

// XY-160D H-Bridge Pins
#define IN1_PIN 4
#define IN2_PIN 5
#define IN3_PIN 6
#define IN4_PIN 7

// PCA9685 Servo Channels
#define SERVO_UP_L 0
#define SERVO_UP_R 1
#define SERVO_DOWN_L 2
#define SERVO_DOWN_R 3
#define SERVO_AXON 0

Adafruit_PWMServoDriver hub_bar = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver hub_gripper = Adafruit_PWMServoDriver(0x60);

bool bar_active = false;
bool gripper_active = false;
unsigned long last_telemetry = 0;

void setServoAngle(Adafruit_PWMServoDriver &hub, uint8_t channel, int angle) {
    int pulse = map(constrain(angle, 0, 180), 0, 180, 150, 600);
    hub.setPWM(channel, 0, pulse);
}

void stopMotors() {
    analogWrite(IN1_PIN, 0);
    analogWrite(IN2_PIN, 0);
    analogWrite(IN3_PIN, 0);
    analogWrite(IN4_PIN, 0);
}

void setMotorDrive(int speed, String dir) {
    speed = constrain(speed, 0, 255);
    if (speed == 0 || dir == "STOP") {
        stopMotors();
        return;
    }

    if (dir == "FORWARD") {
        analogWrite(IN1_PIN, speed);
        analogWrite(IN2_PIN, 0);
        analogWrite(IN3_PIN, speed);
        analogWrite(IN4_PIN, 0);
    } else if (dir == "BACKWARD") {
        analogWrite(IN1_PIN, 0);
        analogWrite(IN2_PIN, speed);
        analogWrite(IN3_PIN, 0);
        analogWrite(IN4_PIN, speed);
    }
}

void executeCommand(String cmd) {
    if (cmd == "NONE") return;

    Serial.print("ACK:");
    Serial.println(cmd);

    if (cmd == "estop") {
        stopMotors();
    } else if (cmd == "up_attach" && bar_active) {
        setServoAngle(hub_bar, SERVO_UP_L, 160);
        setServoAngle(hub_bar, SERVO_UP_R, 160);
    } else if (cmd == "up_detach" && bar_active) {
        setServoAngle(hub_bar, SERVO_UP_L, 20);
        setServoAngle(hub_bar, SERVO_UP_R, 20);
    } else if (cmd == "down_attach" && bar_active) {
        setServoAngle(hub_bar, SERVO_DOWN_L, 160);
        setServoAngle(hub_bar, SERVO_DOWN_R, 160);
    } else if (cmd == "down_detach" && bar_active) {
        setServoAngle(hub_bar, SERVO_DOWN_L, 20);
        setServoAngle(hub_bar, SERVO_DOWN_R, 20);
    } else if (cmd == "both_attach" && bar_active) {
        setServoAngle(hub_bar, SERVO_UP_L, 160);
        setServoAngle(hub_bar, SERVO_UP_R, 160);
        setServoAngle(hub_bar, SERVO_DOWN_L, 160);
        setServoAngle(hub_bar, SERVO_DOWN_R, 160);
    } else if (cmd == "both_detach" && bar_active) {
        setServoAngle(hub_bar, SERVO_UP_L, 20);
        setServoAngle(hub_bar, SERVO_UP_R, 20);
        setServoAngle(hub_bar, SERVO_DOWN_L, 20);
        setServoAngle(hub_bar, SERVO_DOWN_R, 20);
    } else if (cmd == "axon_open" && gripper_active) {
        setServoAngle(hub_gripper, SERVO_AXON, 30);
    } else if (cmd == "axon_close" && gripper_active) {
        setServoAngle(hub_gripper, SERVO_AXON, 150);
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);
    stopMotors();

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setTimeOut(100);

    Wire.beginTransmission(0x40);
    if (Wire.endTransmission() == 0) {
        hub_bar.begin();
        hub_bar.setPWMFreq(50);
        bar_active = true;
    }

    Wire.beginTransmission(0x60);
    if (Wire.endTransmission() == 0) {
        hub_gripper.begin();
        hub_gripper.setPWMFreq(50);
        gripper_active = true;
    }

    digitalWrite(LED_PIN, HIGH); delay(100); digitalWrite(LED_PIN, LOW);

    Serial.println("\n*** ESP32-S3 SYSTEM READY ***");
    Serial.printf("Hub 0x40: %s | Hub 0x60: %s\n",
                  bar_active ? "OK" : "MISSING",
                  gripper_active ? "OK" : "MISSING");
}

void loop() {
    if (Serial.available() > 0) {
        String payload = Serial.readStringUntil('\n');
        payload.trim();

        if (payload.length() > 0) {
            int c1 = payload.indexOf(',');
            int c2 = payload.indexOf(',', c1 + 1);
            int c3 = payload.indexOf(',', c2 + 1);

            if (c1 != -1 && c2 != -1 && c3 != -1) {
                int speed = payload.substring(0, c1).toInt();
                String dir = payload.substring(c1 + 1, c2);
                String cmd = payload.substring(c2 + 1, c3);
                String mode = payload.substring(c3 + 1);

                setMotorDrive(speed, dir);
                executeCommand(cmd);
            }
        }
    }

    if (millis() - last_telemetry > 200) {
        last_telemetry = millis();
        Serial.println("STS:ONLINE");
    }
}