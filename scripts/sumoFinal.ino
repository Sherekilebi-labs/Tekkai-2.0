#include <Arduino.h>



const int trigPins[3] = {27, 25, 32};   // left = D18, middle = TX2, right = D4
const int echoPins[3] = {14, 26, 33};   // left = D19, middle = D5, right = RX2

const int numSensors = sizeof(trigPins) / sizeof(trigPins[0]);

const int remoteOnPin = 23;
const int remoteOffPin = 22;
const int motorDriverEnablePin = 18;
const int motorDriverEnabledState = HIGH;
const int motorDriverDisabledState = LOW;

const int edgeSensorPins[2] = {13, 39};  // 0 = front-left, 1 = front-right
const int edgeSensorDetectedState = LOW;  // white edge detected, black field clear
const int edgeConfirmSamples = 2;
const unsigned long edgeReverseDuration = 500;

const int R_IN1 = 4;
const int R_IN2 = 16;
const int L_IN1 = 21;
const int L_IN2 = 19;

const int defaultRobotSpeed = 255;

#define SOUND_SPEED 0.034
#define DETECTION_THRESHOLD 40.0

long durations[numSensors];
float distances[numSensors];

unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 100;  // 0.1 seconds

int lastDetectedSensor = -1;  // -1 = none
int edgeDetectionCounts[2] = {0, 0};
bool edgeBothReverseActive = false;
unsigned long edgeBothReverseStartTime = 0;

bool started = false;
bool robotPowerEnabled = false;
bool lastRemoteOnHigh = false;
bool lastRemoteOffHigh = false;
bool motorTestActive = false;
bool motorTestPreviousStarted = false;
bool motorTestPreviousPowerEnabled = false;
bool edgeBothSearchActive = false;
unsigned long motorTestStartTime = 0;
const int motorTestPwm = 128;
const unsigned long motorTestPhaseDuration = 2000;

int robotSpeed = defaultRobotSpeed;
int leftMotorPwm = 0;
int rightMotorPwm = 0;


enum EdgeState {
  EDGE_NONE,
  EDGE_LEFT,
  EDGE_RIGHT,
  EDGE_BOTH
};

void turnRobotOn() {
  if (robotPowerEnabled) {
    return;
  }

  robotPowerEnabled = true;
  started = true;
  edgeBothReverseActive = false;
  edgeBothSearchActive = false;
  resetEdgeDetectionCounts();
  lastDetectedSensor = -1;
  lastSampleTime = millis();
  digitalWrite(motorDriverEnablePin, motorDriverEnabledState);
  Serial.println("Remote ON received. Robot algorithm started.");
}

void turnRobotOff() {
  if (!robotPowerEnabled) {
    return;
  }

  motorTestActive = false;
  edgeBothReverseActive = false;
  edgeBothSearchActive = false;
  robotPowerEnabled = false;
  started = false;
  stopMotors();
  resetEdgeDetectionCounts();
  lastDetectedSensor = -1;
  digitalWrite(motorDriverEnablePin, motorDriverDisabledState);
  Serial.println("Remote OFF received. Motor drivers disabled.");
}

void handleRemotePowerControl() {
  bool remoteOnHigh = digitalRead(remoteOnPin) == HIGH;
  bool remoteOffHigh = digitalRead(remoteOffPin) == HIGH;

  if (remoteOffHigh && !lastRemoteOffHigh) {
    turnRobotOff();
  } else if (remoteOnHigh && !lastRemoteOnHigh) {
    turnRobotOn();
  }

  lastRemoteOnHigh = remoteOnHigh;
  lastRemoteOffHigh = remoteOffHigh;
}

void syncRemotePowerState() {
  lastRemoteOnHigh = digitalRead(remoteOnPin) == HIGH;
  lastRemoteOffHigh = digitalRead(remoteOffPin) == HIGH;
}

void startMotorTest() {
  if (motorTestActive) {
    return;
  }

  motorTestPreviousStarted = started;
  motorTestPreviousPowerEnabled = robotPowerEnabled;
  motorTestActive = true;
  edgeBothReverseActive = false;
  edgeBothSearchActive = false;
  motorTestStartTime = millis();
  started = false;
  robotPowerEnabled = true;
  lastDetectedSensor = -1;
  resetEdgeDetectionCounts();
  digitalWrite(motorDriverEnablePin, motorDriverEnabledState);
  stopMotors();
  Serial.println("Motor test started.");
}

void finishMotorTest() {
  stopMotors();
  motorTestActive = false;
  started = motorTestPreviousStarted;
  robotPowerEnabled = motorTestPreviousPowerEnabled;
  digitalWrite(motorDriverEnablePin, robotPowerEnabled ? motorDriverEnabledState : motorDriverDisabledState);
  lastSampleTime = millis();
  Serial.println("Motor test finished.");
}

void updateMotorTest() {
  if (!motorTestActive) {
    return;
  }

  unsigned long elapsed = millis() - motorTestStartTime;

  if (elapsed < motorTestPhaseDuration) {
    driveLeftMotor(motorTestPwm);
    driveRightMotor(0);
  } else if (elapsed < motorTestPhaseDuration * 2) {
    driveLeftMotor(0);
    driveRightMotor(motorTestPwm);
  } else {
    finishMotorTest();
  }
}


void driveMotor(int forwardPin, int reversePin, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    analogWrite(forwardPin, speed);
    analogWrite(reversePin, 0);
  } else if (speed < 0) {
    analogWrite(forwardPin, 0);
    analogWrite(reversePin, -speed);
  } else {
    analogWrite(forwardPin, 0);
    analogWrite(reversePin, 0);
  }
}

void driveLeftMotor(int speed) {
  leftMotorPwm = constrain(speed, -255, 255);
  driveMotor(L_IN1, L_IN2, speed);
}

void driveRightMotor(int speed) {
  rightMotorPwm = constrain(speed, -255, 255);
  driveMotor(R_IN1, R_IN2, speed);
}

void moveLeft() {
  driveLeftMotor((robotSpeed * 20) / 100);
  driveRightMotor(robotSpeed);
}

void moveForward() {
  driveLeftMotor(robotSpeed);
  driveRightMotor(robotSpeed);
}

void moveRight() {
  driveLeftMotor(robotSpeed);
  driveRightMotor((robotSpeed * 20) / 100);
}

void spinTowardRight() {
  driveLeftMotor(robotSpeed);
  driveRightMotor(-robotSpeed);
}

void spinTowardLeft() {
  driveLeftMotor(-robotSpeed);
  driveRightMotor(robotSpeed);
}

void spinRight() {
  spinTowardRight();
}

void spinLeft() {
  spinTowardLeft();
}

void stopMotors() {
  driveLeftMotor(0);
  driveRightMotor(0);
}

void moveBackward() {
  driveLeftMotor(-robotSpeed);
  driveRightMotor(-robotSpeed);
}

void moveLeftCenter() {
  driveLeftMotor((robotSpeed * 60) / 100);
  driveRightMotor(robotSpeed);
}

void moveRightCenter() {
  driveLeftMotor(robotSpeed);
  driveRightMotor((robotSpeed * 60) / 100);
}

void handleOnlyLeftEdge() {
  driveLeftMotor((robotSpeed * 60) / 100);
  driveRightMotor(-robotSpeed);
}

void handleOnlyRightEdge() {
  driveLeftMotor(-robotSpeed);
  driveRightMotor((robotSpeed * 60) / 100);
}

bool isEdgeSensorDetected(int sensorIndex) {
  int sensorValue = digitalRead(edgeSensorPins[sensorIndex]);
  return sensorValue == edgeSensorDetectedState;
}

void resetEdgeDetectionCounts() {
  edgeDetectionCounts[0] = 0;
  edgeDetectionCounts[1] = 0;
}

EdgeState readConfirmedEdgeState() {
  for (int i = 0; i < 2; i++) {
    if (isEdgeSensorDetected(i)) {
      edgeDetectionCounts[i]++;
    } else {
      edgeDetectionCounts[i] = 0;
    }
  }

  bool leftEdge = edgeDetectionCounts[0] >= edgeConfirmSamples;
  bool rightEdge = edgeDetectionCounts[1] >= edgeConfirmSamples;

  if (leftEdge && rightEdge) {
    return EDGE_BOTH;
  } else if (leftEdge) {
    return EDGE_LEFT;
  } else if (rightEdge) {
    return EDGE_RIGHT;
  }

  return EDGE_NONE;
}

void moveNorthWest() {
  moveLeftCenter();
}

void moveNorthEast() {
  moveRightCenter();
}

void callLastDetectedAction() {
  switch (lastDetectedSensor) {
    case 0: spinTowardLeft(); break;
    case 2: spinTowardRight(); break;
    default: spinTowardLeft(); break;
  }
}

bool ultrasonicDetected(int sensorIndex) {
  return durations[sensorIndex] > 0 && distances[sensorIndex] < DETECTION_THRESHOLD;
}

int readTargetMask() {
  int mask = 0;
  if (ultrasonicDetected(0)) mask |= 0b001;
  if (ultrasonicDetected(1)) mask |= 0b010;
  if (ultrasonicDetected(2)) mask |= 0b100;
  return mask;
}

void driveTowardTarget(int targetMask) {
  edgeBothReverseActive = false;
  edgeBothSearchActive = false;

  switch (targetMask) {
    case 0b001:  // left only
      lastDetectedSensor = 0;
      moveLeft();
      break;
    case 0b011:  // left + center
      lastDetectedSensor = 0;
      moveLeftCenter();
      break;
    case 0b010:  // center only
    case 0b111:  // all sensors
      lastDetectedSensor = 1;
      moveForward();
      break;
    case 0b100:  // right only
      lastDetectedSensor = 2;
      moveRight();
      break;
    case 0b110:  // center + right
      lastDetectedSensor = 2;
      moveRightCenter();
      break;
    case 0b101:  // left + right without center; treat as a wide/close target
      lastDetectedSensor = 1;
      moveForward();
      break;
    default:
      callLastDetectedAction();
      break;
  }
}

void startBothEdgeReverse() {
  edgeBothReverseActive = true;
  edgeBothSearchActive = false;
  edgeBothReverseStartTime = millis();
  lastDetectedSensor = -1;
  moveBackward();
}

bool updateBothEdgeReverse() {
  if (edgeBothSearchActive) {
    spinTowardLeft();
    return true;
  }

  if (!edgeBothReverseActive) {
    return false;
  }

  if (millis() - edgeBothReverseStartTime < edgeReverseDuration) {
    moveBackward();
  } else {
    edgeBothReverseActive = false;
    edgeBothSearchActive = true;
    spinTowardLeft();
  }

  return true;
}

void handleEdgeWithoutTarget(EdgeState edgeState) {
  if (edgeBothSearchActive) {
    spinTowardLeft();
    return;
  }

  switch (edgeState) {
    case EDGE_LEFT:
      edgeBothReverseActive = false;
      handleOnlyLeftEdge();
      break;
    case EDGE_RIGHT:
      edgeBothReverseActive = false;
      handleOnlyRightEdge();
      break;
    case EDGE_BOTH:
      if (!edgeBothReverseActive) {
        startBothEdgeReverse();
      } else {
        updateBothEdgeReverse();
      }
      break;
    case EDGE_NONE:
    default:
      if (!updateBothEdgeReverse()) {
        callLastDetectedAction();
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize ultrasonic pins
  for (int i = 0; i < numSensors; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);   
  pinMode(motorDriverEnablePin, OUTPUT);
  digitalWrite(motorDriverEnablePin, motorDriverDisabledState);
  pinMode(edgeSensorPins[0], INPUT_PULLUP);
  pinMode(edgeSensorPins[1], INPUT);
  stopMotors();

  pinMode(remoteOnPin, INPUT_PULLDOWN);
  pinMode(remoteOffPin, INPUT_PULLDOWN);
  lastRemoteOnHigh = digitalRead(remoteOnPin) == HIGH;
  lastRemoteOffHigh = digitalRead(remoteOffPin) == HIGH;
  Serial.println("Robot starts OFF. Remote ON signal on pin 23 can enable it.");
  Serial.println("Use the Serial Monitor to adjust motor speed (0-255).");
}

void loop() {
  handleRemotePowerControl();

  if (motorTestActive) {
    updateMotorTest();
    return;
  }

  if (!started || !robotPowerEnabled) {
    stopMotors();
    return;
  }

  unsigned long currentTime = millis();
  if (currentTime - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentTime;

    for (int i = 0; i < numSensors; i++) {
      digitalWrite(trigPins[i], LOW);
      delayMicroseconds(2);
      digitalWrite(trigPins[i], HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPins[i], LOW);

      handleRemotePowerControl();
      if (!robotPowerEnabled) {
        return;
      }

      durations[i] = pulseIn(echoPins[i], HIGH, 25000);
      distances[i] = durations[i] * SOUND_SPEED / 2;

      Serial.print("Distance ");
      Serial.print(i);
      Serial.print(" (cm): ");
      Serial.println(distances[i]);  // delete after testing
    }

    // Serial input for speed adjustment
    if (Serial.available() > 0) {
      int inputSpeed = Serial.parseInt();
      if (inputSpeed >= 0 && inputSpeed <= 255) {
        robotSpeed = inputSpeed;
        Serial.print("Robot speed set to: ");
        Serial.println(robotSpeed);
      }
    }

    int targetMask = readTargetMask();
    if (targetMask != 0) {
      driveTowardTarget(targetMask);
      resetEdgeDetectionCounts();
      return;
    }

    EdgeState edgeState = readConfirmedEdgeState();
    handleEdgeWithoutTarget(edgeState);
  }
}
