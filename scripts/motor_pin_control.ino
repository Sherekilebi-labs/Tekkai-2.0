#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

const int L_IN1 = 4;
const int L_IN2 = 16;
const int R_IN1 = 19;
const int R_IN2 = 21;

const int motorDriverEnablePin = 18;
const int motorDriverEnabledState = HIGH;
const int motorDriverDisabledState = LOW;

const char* debugApSsid = "SUMO_MOTOR_TEST";
const char* debugApPassword = "";

WebServer server(80);

const int motorPins[] = {L_IN1, L_IN2, R_IN1, R_IN2};
const char* motorPinNames[] = {"L_IN1 GPIO4", "L_IN2 GPIO16", "R_IN1 GPIO19", "R_IN2 GPIO21"};
const int motorPinCount = sizeof(motorPins) / sizeof(motorPins[0]);

int dutyPercent[motorPinCount] = {0, 0, 0, 0};
bool driverEnabled = false;

const char DEBUG_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Motor Pin Control</title>
  <style>
    body { margin: 0; padding: 20px; background: #101114; color: #f4f4f5; font-family: system-ui, -apple-system, sans-serif; }
    main { max-width: 720px; margin: 0 auto; }
    h1 { margin: 0 0 16px; font-size: 1.4rem; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 12px; }
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
    .slider { display: grid; gap: 10px; }
    input[type="range"] { width: 100%; accent-color: #22c55e; }
    .msg { margin: 12px 0; min-height: 20px; color: #a1a1aa; font: 700 0.8rem ui-monospace, monospace; }
  </style>
</head>
<body>
  <main>
    <h1>Motor Pin Control</h1>
    <div class="row">
      <form action="/enable" method="get"><button class="on" type="submit">Enable Drivers</button></form>
      <form action="/disable" method="get"><button class="off" type="submit">Disable Drivers</button></form>
    </div>
    <div class="msg" id="msg"></div>
    <div class="grid">
      <div class="card"><div class="label">Driver State</div><div class="value" id="driver">-</div></div>
      <div class="card"><div class="label">Enable GPIO18</div><div class="value" id="enablePin">-</div></div>
      <div class="card slider">
        <div class="label">L_IN1 GPIO4</div>
        <div class="value"><span id="v0">0</span>%</div>
        <input id="s0" type="range" min="0" max="100" value="0" oninput="setDuty(0, this.value)">
      </div>
      <div class="card slider">
        <div class="label">L_IN2 GPIO16</div>
        <div class="value"><span id="v1">0</span>%</div>
        <input id="s1" type="range" min="0" max="100" value="0" oninput="setDuty(1, this.value)">
      </div>
      <div class="card slider">
        <div class="label">R_IN1 GPIO19</div>
        <div class="value"><span id="v2">0</span>%</div>
        <input id="s2" type="range" min="0" max="100" value="0" oninput="setDuty(2, this.value)">
      </div>
      <div class="card slider">
        <div class="label">R_IN2 GPIO21</div>
        <div class="value"><span id="v3">0</span>%</div>
        <input id="s3" type="range" min="0" max="100" value="0" oninput="setDuty(3, this.value)">
      </div>
    </div>
  </main>
  <script>
    var pending = {};

    function pill(active, activeText, inactiveText) {
      return '<span class="pill ' + (active ? 'high' : 'low') + '">' + (active ? activeText : inactiveText) + '</span>';
    }

    function render(d) {
      document.getElementById('driver').innerHTML = pill(d.driverEnabled, 'ENABLED', 'DISABLED');
      document.getElementById('enablePin').innerHTML = pill(d.enablePinHigh, 'HIGH', 'LOW');

      for (var i = 0; i < d.duty.length; i++) {
        document.getElementById('v' + i).textContent = d.duty[i];
        if (document.activeElement !== document.getElementById('s' + i)) {
          document.getElementById('s' + i).value = d.duty[i];
        }
      }
    }

    function setDuty(pin, duty) {
      duty = Math.max(0, Math.min(100, parseInt(duty, 10) || 0));
      document.getElementById('v' + pin).textContent = duty;

      clearTimeout(pending[pin]);
      pending[pin] = setTimeout(function() {
        fetch('/set?pin=' + pin + '&duty=' + duty + '&t=' + Date.now(), { cache: 'no-store' })
          .then(function(r) { return r.json(); })
          .then(render)
          .catch(function(e) {
            document.getElementById('msg').textContent = 'Set failed: ' + e;
          });
      }, 40);
    }

    function update() {
      fetch('/status?t=' + Date.now(), { cache: 'no-store' })
        .then(function(r) { return r.json(); })
        .then(render)
        .catch(function() {});
    }

    update();
    setInterval(update, 500);
  </script>
</body>
</html>
)rawliteral";

void applyOutputs() {
  if (!driverEnabled) {
    for (int i = 0; i < motorPinCount; i++) {
      analogWrite(motorPins[i], 0);
    }
    digitalWrite(motorDriverEnablePin, motorDriverDisabledState);
    return;
  }

  digitalWrite(motorDriverEnablePin, motorDriverEnabledState);
  for (int i = 0; i < motorPinCount; i++) {
    int pwm = map(dutyPercent[i], 0, 100, 0, 255);
    analogWrite(motorPins[i], pwm);
  }
}

void sendStatus() {
  String json = "{\"driverEnabled\":" + String(driverEnabled ? "true" : "false") +
                ",\"enablePinHigh\":" + String(digitalRead(motorDriverEnablePin) == HIGH ? "true" : "false") +
                ",\"pins\":[";

  for (int i = 0; i < motorPinCount; i++) {
    json += "{\"name\":\"" + String(motorPinNames[i]) +
            "\",\"gpio\":" + String(motorPins[i]) +
            ",\"pwm\":" + String(map(dutyPercent[i], 0, 100, 0, 255)) + "}";
    if (i < motorPinCount - 1) json += ",";
  }

  json += "],\"duty\":[";
  for (int i = 0; i < motorPinCount; i++) {
    json += String(dutyPercent[i]);
    if (i < motorPinCount - 1) json += ",";
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
    sendStatus();
  });

  server.on("/set", []() {
    int pinIndex = server.arg("pin").toInt();
    int duty = server.arg("duty").toInt();

    if (pinIndex < 0 || pinIndex >= motorPinCount) {
      server.send(400, "text/plain", "Invalid pin index");
      return;
    }

    dutyPercent[pinIndex] = constrain(duty, 0, 100);
    applyOutputs();
    sendStatus();
  });

  server.on("/enable", []() {
    driverEnabled = true;
    applyOutputs();
    redirectToDebugPage();
  });

  server.on("/disable", []() {
    driverEnabled = false;
    applyOutputs();
    redirectToDebugPage();
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

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < motorPinCount; i++) {
    pinMode(motorPins[i], OUTPUT);
    analogWrite(motorPins[i], 0);
  }

  pinMode(motorDriverEnablePin, OUTPUT);
  digitalWrite(motorDriverEnablePin, motorDriverDisabledState);

  setupDebugServer();
  Serial.println("Motor drivers start DISABLED. Press Enable Drivers on the web page.");
}

void loop() {
  server.handleClient();
}
