#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

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

const char* debugApSsid = "SUMO_2026_DEBUG";
const char* debugApPassword = "";
WebServer server(80);

const char DEBUG_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Sumo 2026 Debug</title>
  <style>
    body { margin: 0; padding: 20px; background: #101114; color: #f4f4f5; font-family: system-ui, -apple-system, sans-serif; }
    main { max-width: 720px; margin: 0 auto; }
    h1 { margin: 0 0 16px; font-size: 1.4rem; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(160px, 1fr)); gap: 12px; }
    .card { background: #181a20; border: 1px solid #2e323b; border-radius: 8px; padding: 14px; }
    .label { color: #a1a1aa; font-size: 0.75rem; text-transform: uppercase; letter-spacing: 0.04em; }
    .value { margin-top: 6px; font: 700 1.4rem ui-monospace, SFMono-Regular, Menlo, monospace; }
    .row { display: flex; gap: 10px; margin: 16px 0; }
    .row form { flex: 1; display: flex; margin: 0; }
    button { width: 100%; border: 0; border-radius: 6px; padding: 14px; font-weight: 800; cursor: pointer; }
    .on { background: #22c55e; color: #052e16; }
    .off { background: #ef4444; color: #450a0a; }
    .pill { display: inline-block; padding: 4px 9px; border-radius: 999px; font: 700 0.75rem ui-monospace, monospace; }
    .high { background: #064e3b; color: #6ee7b7; }
    .low { background: #3f3f46; color: #d4d4d8; }
    .msg { margin: 12px 0; min-height: 20px; color: #a1a1aa; font: 700 0.8rem ui-monospace, monospace; }
  </style>
</head>
<body>
  <main>
    <h1>Sumo 2026 Debug</h1>
    <div class="row">
      <form action="/on" method="get"><button class="on" type="submit">Manual ON</button></form>
      <form action="/off" method="get"><button class="off" type="submit">Manual OFF</button></form>
    </div>
    <div class="row">
      <form action="/motor-test" method="get"><button class="on" type="submit">Motor Test</button></form>
    </div>
    <div class="msg" id="msg"></div>
    <div class="grid">
      <div class="card"><div class="label">Robot State</div><div class="value" id="state">-</div></div>
      <div class="card"><div class="label">Motor Test</div><div class="value" id="motorTest">-</div></div>
      <div class="card"><div class="label">Driver Enable GPIO18</div><div class="value" id="enable">-</div></div>
      <div class="card"><div class="label">Remote ON GPIO23</div><div class="value" id="remoteOn">-</div></div>
      <div class="card"><div class="label">Remote OFF GPIO22</div><div class="value" id="remoteOff">-</div></div>
      <div class="card"><div class="label">Ultrasonic Left</div><div class="value"><span id="dist0">-</span> cm</div></div>
      <div class="card"><div class="label">Ultrasonic Middle</div><div class="value"><span id="dist1">-</span> cm</div></div>
      <div class="card"><div class="label">Ultrasonic Right</div><div class="value"><span id="dist2">-</span> cm</div></div>
      <div class="card"><div class="label">Edge Front Left</div><div class="value" id="edge0">-</div></div>
      <div class="card"><div class="label">Edge Front Right</div><div class="value" id="edge1">-</div></div>
      <div class="card"><div class="label">Speed</div><div class="value" id="speed">-</div></div>
      <div class="card"><div class="label">Left Motor PWM</div><div class="value"><span id="leftMotor">-</span>%</div></div>
      <div class="card"><div class="label">Right Motor PWM</div><div class="value"><span id="rightMotor">-</span>%</div></div>
    </div>
  </main>
  <script>
    function pill(active, activeText, inactiveText) {
      return '<span class="pill ' + (active ? 'high' : 'low') + '">' + (active ? activeText : inactiveText) + '</span>';
    }

    function render(d) {
      document.getElementById('state').innerHTML = pill(d.enabled, 'ON', 'OFF');
      document.getElementById('motorTest').innerHTML = pill(d.motorTest.active, d.motorTest.phase, 'IDLE');
      document.getElementById('enable').innerHTML = pill(d.driverEnable, 'HIGH', 'LOW');
      document.getElementById('remoteOn').innerHTML = pill(d.remoteOn, 'HIGH', 'LOW');
      document.getElementById('remoteOff').innerHTML = pill(d.remoteOff, 'HIGH', 'LOW');
      document.getElementById('dist0').textContent = d.distances[0].toFixed(1);
      document.getElementById('dist1').textContent = d.distances[1].toFixed(1);
      document.getElementById('dist2').textContent = d.distances[2].toFixed(1);
      document.getElementById('edge0').innerHTML = pill(d.edges[0], 'DETECTED', 'CLEAR');
      document.getElementById('edge1').innerHTML = pill(d.edges[1], 'DETECTED', 'CLEAR');
      document.getElementById('speed').textContent = d.speed;
      document.getElementById('leftMotor').textContent = d.motors.leftPct;
      document.getElementById('rightMotor').textContent = d.motors.rightPct;
    }

    function update() {
      fetch('/status?t=' + Date.now(), { cache: 'no-store' })
        .then(function(r) { return r.json(); })
        .then(render)
        .catch(function() {});
    }

    update();
    setInterval(update, 200);
  </script>
</body>
</html>
)rawliteral";

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

String getMotorTestPhase() {
  if (!motorTestActive) {
    return "IDLE";
  }

  unsigned long elapsed = millis() - motorTestStartTime;
  if (elapsed < motorTestPhaseDuration) {
    return "LEFT 50%";
  }
  if (elapsed < motorTestPhaseDuration * 2) {
    return "RIGHT 50%";
  }
  return "ENDING";
}

void sendDebugStatus() {
  int leftMotorPct = map(leftMotorPwm, -255, 255, -100, 100);
  int rightMotorPct = map(rightMotorPwm, -255, 255, -100, 100);

  String json = "{\"enabled\":" + String(robotPowerEnabled ? "true" : "false") +
                ",\"started\":" + String(started ? "true" : "false") +
                ",\"motorTest\":{\"active\":" + String(motorTestActive ? "true" : "false") +
                ",\"phase\":\"" + getMotorTestPhase() + "\"}" +
                ",\"driverEnable\":" + String(digitalRead(motorDriverEnablePin) == HIGH ? "true" : "false") +
                ",\"remoteOn\":" + String(digitalRead(remoteOnPin) == HIGH ? "true" : "false") +
                ",\"remoteOff\":" + String(digitalRead(remoteOffPin) == HIGH ? "true" : "false") +
                ",\"speed\":" + String(robotSpeed) +
                ",\"motors\":{\"leftPwm\":" + String(leftMotorPwm) +
                ",\"rightPwm\":" + String(rightMotorPwm) +
                ",\"leftPct\":" + String(leftMotorPct) +
                ",\"rightPct\":" + String(rightMotorPct) + "}" +
                ",\"distances\":[";

  for (int i = 0; i < numSensors; i++) {
    json += String(distances[i], 1);
    if (i < numSensors - 1) json += ",";
  }

  json += "],\"edges\":[";
  for (int i = 0; i < 2; i++) {
    json += isEdgeSensorDetected(i) ? "true" : "false";
    if (i < 1) json += ",";
  }

  json += "]}";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void redirectToDebugPage() {
  server.sendHeader("Location", "/", true);
  server.sendHeader("Cache-Control", "no-store");
  server.send(303, "text/plain", "");
}

void setupDebugServer() {
  WiFi.mode(WIFI_AP);
  if (strlen(debugApPassword) >= 8) {
    WiFi.softAP(debugApSsid, debugApPassword);
  } else {
    WiFi.softAP(debugApSsid);
  }

  server.on("/", []() {
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "text/html", DEBUG_PAGE);
  });

  server.on("/status", []() {
    sendDebugStatus();
  });

  server.on("/on", []() {
    turnRobotOn();
    syncRemotePowerState();
    redirectToDebugPage();
  });

  server.on("/on", HTTP_POST, []() {
    turnRobotOn();
    syncRemotePowerState();
    sendDebugStatus();
  });

  server.on("/off", []() {
    turnRobotOff();
    syncRemotePowerState();
    redirectToDebugPage();
  });

  server.on("/off", HTTP_POST, []() {
    turnRobotOff();
    syncRemotePowerState();
    sendDebugStatus();
  });

  server.on("/motor-test", []() {
    startMotorTest();
    syncRemotePowerState();
    redirectToDebugPage();
  });

  server.on("/motor-test", HTTP_POST, []() {
    startMotorTest();
    syncRemotePowerState();
    sendDebugStatus();
  });

  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found: " + server.uri());
  });

  server.begin();
  Serial.print("Debug Wi-Fi SSID: ");
  Serial.println(debugApSsid);
  Serial.print("Debug page: http://");
  Serial.println(WiFi.softAPIP());
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
  setupDebugServer();
  Serial.println("Robot starts OFF. Remote ON signal on pin 23 can enable it.");
  Serial.println("Use the Serial Monitor to adjust motor speed (0-255).");
}

void loop() {
  server.handleClient();
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
