/*
  Arm_Code_1_1_serial_pca9685.ino

  Receives joint angles from Python over USB serial in this format:

      theta1,theta2,theta3,theta4,gripper

  Example:
      90,120,280,300,90

  Hardware:
  - Arduino UNO R3
  - PCA9685 servo driver
  - 4 arm servos now + 5th gripper servo
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "OV7670_UART.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// -------------------------
// PCA9685 channel assignment
// -------------------------
const uint8_t BASE_CH     = 0;
const uint8_t SHOULDER_CH = 1;
const uint8_t ELBOW_CH    = 2;
const uint8_t WRIST_CH    = 3;
const uint8_t GRIPPER_CH  = 4;   // optional 5th servo

// -------------------------
// Servo pulse calibration
// Start with these values. Adjust if your servos do not reach 0/180 correctly.
// TowerPro SG90-type servos commonly work around 500-2500 us.
// -------------------------
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const int SERVO_FREQ   = 50;

// ------------------------
// Servo speed controls
//const int SERVO_FREQ = 50;
const float ARM_SPEED_DPS = 10.0;      // Maximum arm joint speed in degrees per second
const float GRIPPER_SPEED_DPS = 5.0;   // Tested gripper speed in degrees per second


// -------------------------
// Safe physical servo limits.
// Keep conservative first, then widen after testing each joint.
// These are servo-command angles, not IK model angles.
// -------------------------
const int BASE_MIN_SERVO     = 0;
const int BASE_MAX_SERVO     = 180;
const int SHOULDER_MIN_SERVO = 30;
const int SHOULDER_MAX_SERVO = 150;
const int ELBOW_MIN_SERVO    = 0;
const int ELBOW_MAX_SERVO    = 180;
const int WRIST_MIN_SERVO    = 0;
const int WRIST_MAX_SERVO    = 180;
const int GRIPPER_MIN_SERVO  = 54;
const int GRIPPER_MAX_SERVO  = 73;

// Tested gripper positions
const int GRIPPER_CLOSED_ANGLE = 54;
const int GRIPPER_START_ANGLE  = 70;
const int GRIPPER_OPEN_ANGLE   = 73;

// -------------------------
// Calibration offsets and direction reversals.
// Change these after testing.
// If a joint moves the wrong way, change its REVERSE value.
// If zero position is wrong, adjust its OFFSET.
// -------------------------
const bool BASE_REVERSE     = false;
const bool SHOULDER_REVERSE = true;
const bool ELBOW_REVERSE    = true;
const bool WRIST_REVERSE    = true;
const bool GRIPPER_REVERSE  = false;

const int BASE_OFFSET     = -5;
const int SHOULDER_OFFSET = 14;
const int ELBOW_OFFSET    = 12;
const int WRIST_OFFSET    = 17;
const int GRIPPER_OFFSET  = 0;

// -------------------------
// Logical start position
// -------------------------
const int START_BASE     = 90;
const int START_SHOULDER = 110;
const int START_ELBOW    = 15;
const int START_WRIST    = 15;
const int START_GRIPPER  = 70;

// Current servo positions
int baseServo     = 90;
int shoulderServo = 90;
int elbowServo    = 90;
int wristServo    = 90;
int gripperServo  = 90;

// Target servo positions
int baseTarget     = 90;
int shoulderTarget = 90;
int elbowTarget    = 90;
int wristTarget    = 90;
int gripperTarget  = 90;

unsigned long lastArmStepTime = 0;
unsigned long lastGripperStepTime = 0;

// Serial input buffer
String inputLine = "";

void setup() {
  Serial.begin(115200);
  Wire.begin();

  initializeScreenAndCamera();
  pinMode(8, INPUT_PULLUP);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

// Initialise the servos at the known 90-degree startup position
// Convert the logical 90-degree start position
// through the same calibration used during normal operation
baseServo     = modelBaseToServo(START_BASE);
shoulderServo = modelShoulderToServo(START_SHOULDER);
elbowServo    = modelElbowToServo(START_ELBOW);
wristServo    = modelWristToServo(START_WRIST);
gripperServo  = START_GRIPPER;

// Make the targets match the actual startup positions
baseTarget     = baseServo;
shoulderTarget = shoulderServo;
elbowTarget    = elbowServo;
wristTarget    = wristServo;
gripperTarget  = gripperServo;

// Now initialise the PCA9685
writeServoAngle(BASE_CH,     baseServo);
writeServoAngle(SHOULDER_CH, shoulderServo);
writeServoAngle(ELBOW_CH,    elbowServo);
writeServoAngle(WRIST_CH,    wristServo);
writeServoAngle(GRIPPER_CH,  gripperServo);

  Serial.println("READY");
}

void loop() {
  readSerialCommand();
  updateArmMotion();
  updateGripperMotion();
}

void readSerialCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      inputLine.trim();
      if (inputLine.length() > 0) {
        if (inputLine.equalsIgnoreCase("CAM")) {
            processFrame();
        }
        else {
            parseAndMove(inputLine);
        }
    }
      inputLine = "";
    } 
    else {
      inputLine += c;
      if (inputLine.length() > 80) {
        Serial.println("ERR input line too long, clearing buffer");
        inputLine = "";
      }
    }
  }
}

void parseAndMove(String line) {
  float values[5] = {0, 0, 0, 0, 0};
  int valueIndex = 0;
  int startIndex = 0;

  line.trim();

  for (int i = 0; i <= line.length(); i++) {
    if (i == line.length() || line.charAt(i) == ',') {
      if (valueIndex >= 5) {
        Serial.print("ERR too many values: ");
        Serial.println(line);
        return;
      }

      String token = line.substring(startIndex, i);
      token.trim();

      if (token.length() == 0) {
        Serial.print("ERR empty value: ");
        Serial.println(line);
        return;
      }

      values[valueIndex] = token.toFloat();
      valueIndex++;
      startIndex = i + 1;
    }
  }

  if (valueIndex != 5) {
    Serial.print("ERR expected exactly 5 values: ");
    Serial.println(line);
    return;
  }

  // Reject impossible values before moving anything
  if (values[0] <= 0  ||  values[0] >= 180 ||
      values[1] <= 30 ||  values[1] >= 150 ||
      values[2] <= 0  ||  values[2] >= 90  ||
      values[3] <= 0  ||  values[3] >= 90) {
    Serial.print("ERR servo angle out of range: ");
    Serial.println(line);
    return;
  }

if (values[4] < GRIPPER_CLOSED_ANGLE ||
    values[4] > GRIPPER_OPEN_ANGLE) {

    Serial.print("ERR gripper angle out of range: ");
    Serial.println(values[4]);
    return;
}

  int baseCmd     = modelBaseToServo(values[0]);
  int shoulderCmd = modelShoulderToServo(values[1]);
  int elbowCmd    = modelElbowToServo(values[2]);
  int wristCmd    = modelWristToServo(values[3]);
  int gripperCmd = round(values[4]);

  moveArmServos(baseCmd, shoulderCmd, elbowCmd, wristCmd, gripperCmd);

  Serial.print("OK RAW ");
  Serial.print(values[0]); Serial.print(',');
  Serial.print(values[1]); Serial.print(',');
  Serial.print(values[2]); Serial.print(',');
  Serial.print(values[3]); Serial.print(',');
  Serial.print(values[4]);

  Serial.print(" CMD ");
  Serial.print(baseCmd); Serial.print(',');
  Serial.print(shoulderCmd); Serial.print(',');
  Serial.print(elbowCmd); Serial.print(',');
  Serial.print(wristCmd); Serial.print(',');
  Serial.println(gripperCmd);
}


// ------------------------------------------------------------
// IMPORTANT CALIBRATION SECTION
//
// Your Python model uses mathematical joint angles such as:
// theta1: 0-360
// theta2: 30-150
// theta3: 200-360
// theta4: wrap-around, e.g. 200 through 360 then 0 through 160
//
// Standard micro servos normally accept only about 0-180 degrees.
// Therefore these functions convert model angles into servo commands.
// Tune these mappings to match your real mechanical build.
// ------------------------------------------------------------

int modelBaseToServo(float theta) {
  // Model base 0-360 maps to servo 0-180.
  // This means the real base can cover 180 degrees unless you use a continuous/270/360 servo.
  int servo = round(theta);
  return calibrateServo(servo, BASE_REVERSE, BASE_OFFSET, BASE_MIN_SERVO, BASE_MAX_SERVO);
}

int modelShoulderToServo(float theta) {
  // Python shoulder is already roughly within 30-150, so use it directly first.
  int servo = round(theta);
  return calibrateServo(servo, SHOULDER_REVERSE, SHOULDER_OFFSET, SHOULDER_MIN_SERVO, SHOULDER_MAX_SERVO);
}

int modelElbowToServo(float theta) {
  // Python elbow currently uses 200-360.
  // Convert 200->0 and 360->160 as a starting point.
  int servo = round(theta);
  return calibrateServo(servo, ELBOW_REVERSE, ELBOW_OFFSET, ELBOW_MIN_SERVO, ELBOW_MAX_SERVO);
}

int modelWristToServo(float theta) {
  // Python wrist is wrap-around: 200..360 and 0..160.
  // Convert 200->0, 300->100, 360/0->160, 160->320 then clamp.
  // This is only a safe starting map; tune after checking the real wrist linkage.
  int servo = round(theta);
  return calibrateServo(servo, WRIST_REVERSE, WRIST_OFFSET, WRIST_MIN_SERVO, WRIST_MAX_SERVO);
}

int modelGripperToServo(float theta) {
  int servo = round(theta);
  return calibrateServo(servo, GRIPPER_REVERSE, GRIPPER_OFFSET, GRIPPER_MIN_SERVO, GRIPPER_MAX_SERVO);
}

int calibrateServo(int angle, bool reverse, int offset, int minAngle, int maxAngle) {
  angle = angle + offset;

  if (reverse) {
    angle = 180 - angle;
  }

  angle = constrain(angle, minAngle, maxAngle);
  return angle;
}

void moveArmServos(int baseAngle, int shoulderAngle, int elbowAngle, int wristAngle, int gripperAngle) {
  // Only update the target positions.
  // updateArmMotion() will move towards them gradually.

  baseTarget     = baseAngle;
  shoulderTarget = shoulderAngle;
  elbowTarget    = elbowAngle;
  wristTarget    = wristAngle;
  gripperTarget  = gripperAngle;
}

void updateArmMotion() {

  unsigned long stepInterval = (unsigned long)(1000.0 / ARM_SPEED_DPS);

  if (millis() - lastArmStepTime < stepInterval) {
    return;
  }

  lastArmStepTime = millis();

  // Move each joint by a maximum of 1 degree per step

  if (baseServo < baseTarget) {
    baseServo++;
  }
  else if (baseServo > baseTarget) {
    baseServo--;
  }

  if (shoulderServo < shoulderTarget) {
    shoulderServo++;
  }
  else if (shoulderServo > shoulderTarget) {
    shoulderServo--;
  }

  if (elbowServo < elbowTarget) {
    elbowServo++;
  }
  else if (elbowServo > elbowTarget) {
    elbowServo--;
  }

  if (wristServo < wristTarget) {
    wristServo++;
  }
  else if (wristServo > wristTarget) {
    wristServo--;
  }

  writeServoAngle(BASE_CH,     baseServo);
  writeServoAngle(SHOULDER_CH, shoulderServo);
  writeServoAngle(ELBOW_CH,    elbowServo);
  writeServoAngle(WRIST_CH,    wristServo);
}

void updateGripperMotion() {
  unsigned long stepInterval = (unsigned long)(1000.0 / GRIPPER_SPEED_DPS);

  if (millis() - lastGripperStepTime < stepInterval) {
    return;
  }

  lastGripperStepTime = millis();

  if (gripperServo < gripperTarget) {
    gripperServo++;
  }
  else if (gripperServo > gripperTarget) {
    gripperServo--;
  }

  gripperServo = constrain(
    gripperServo,
    GRIPPER_MIN_SERVO,
    GRIPPER_MAX_SERVO
  );

  writeServoAngle(GRIPPER_CH, gripperServo);
}


void writeServoAngle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);

  int pulseUs = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);

  // PCA9685 has 4096 counts per 20 ms period at 50 Hz.
  int pulseCounts = round(pulseUs * 4096.0 / 20000.0);

  pwm.setPWM(channel, 0, pulseCounts);
}
