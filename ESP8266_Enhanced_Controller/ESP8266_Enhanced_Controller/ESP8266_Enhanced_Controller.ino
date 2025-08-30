/*
  ESP8266 Enhanced Controller for STM32_Enhanced_LineFollower
  - Hosts WiFi AP and web UI
  - Sends commands to STM32 over Serial
  - EEPROM persistence for PID, speed, thrust, thruster enable
  - Auto-applies saved settings on boot

  Serial commands sent to STM32:
  start | stop | Kp=val | Ki=val | Kd=val | speed=val | thrust=val | thruster=on|off | pid | status
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
// #include <ESP8266mDNS.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

// AP configuration
const char* ap_ssid = "FIBEE";
const char* ap_password = "oxitech@1"; // min 8 chars
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// EEPROM
#define EEPROM_SIZE 512
#define SETTINGS_ADDRESS 0

struct Settings {
  float Kp;
  float Ki;
  float Kd;
  int baseSpeed;
  bool valid;
};

// Web server
ESP8266WebServer server(80);

// Robot status cache
float gKp = 10.0f;
float gKi = 8.0f;
float gKd = 25.0f;
int gBaseSpeed = 220;
bool gLineFollowing = false;
int gLinePosition = 0;
float gPidOutput = 0.0f;
// Manual drive
volatile int joyX = 0; // -100..100 right positive
volatile int joyY = 0; // -100..100 forward positive


#define SERIAL_BAUD 115200
#define LED_PIN 2

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  EEPROM.begin(EEPROM_SIZE);
  loadSettings();

  // WiFi AP Setup
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  
  if (WiFi.softAP(ap_ssid, ap_password)) {
    Serial.println("WiFi AP started successfully");
    Serial.print("SSID: ");
    Serial.println(ap_ssid);
    Serial.print("Password: ");
    Serial.println(ap_password);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
    digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.println("Failed to start WiFi AP");
    digitalWrite(LED_PIN, LOW);
  }
  
  delay(2000); // Give AP time to start

  setupWebServer();
  server.begin();
  Serial.println("Web server started");

  // Apply persisted settings to STM32
  delay(800);
  sendSettingsToSTM32();
}

void loop() {
  server.handleClient();
  
  // Monitor WiFi AP status
  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck >= 5000) { // Check every 5 seconds
    lastWiFiCheck = millis();
    if (WiFi.softAPgetStationNum() > 0) {
      Serial.print("Connected devices: ");
      Serial.println(WiFi.softAPgetStationNum());
    }
  }

  // Handle inbound status from STM32
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    if (data.length() > 0) parseSTM32Data(data);
  }

  // Periodic status request
  static unsigned long lastReq = 0;
  if (millis() - lastReq >= 800) {
    lastReq = millis();
    sendCommand("status");
  }
}

void setupWebServer() {
  server.on("/", HTTP_GET, [](){
    String html = R"html(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Enhanced Line Follower</title>
  <style>
    body { font-family: Arial, sans-serif; background:#f6f7fb; margin:0; }
    .container { max-width: 900px; margin: 20px auto; background: #fff; padding: 20px; border-radius: 10px; box-shadow: 0 4px 14px rgba(0,0,0,.08); }
    h2 { margin-top:0; }
    .row { display:flex; gap:16px; flex-wrap:wrap; }
    .card { flex:1 1 260px; background:#fafafa; padding:16px; border-radius:8px; }
    .btn { padding:10px 16px; border:none; border-radius:6px; background:#1976d2; color:#fff; cursor:pointer; margin:4px; }
    .btn-danger { background:#c62828; }
    .input { padding:6px 10px; border:1px solid #ddd; border-radius:6px; width:120px; }
    label { display:inline-block; width:120px; }
    .kv { margin:6px 0; }
    .joystick { width: 220px; height: 220px; border-radius: 50%; background: #eef2f7; position: relative; touch-action: none; }
    .stick { width: 70px; height: 70px; border-radius: 50%; background: #90caf9; position: absolute; left: 75px; top: 75px; transform: translate(0,0); }
  </style>
</head>
<body>
  <div class="container">
    <h2>Enhanced Line Follower Control</h2>

    <div class="row">
      <div class="card">
        <h3>Run Control</h3>
        <button class="btn" onclick="send('manual')">Manual</button>
        <button class="btn" onclick="send('auto')">Auto</button>
        <button class="btn btn-danger" onclick="send('stopdc')">Stop DC</button>
      </div>

      <div class="card">
        <h3>Joystick</h3>
        <div id="joy" class="joystick"><div id="stick" class="stick"></div></div>
      </div>

      <div class="card">
        <h3>PID</h3>
        <div class="kv"><label>Kp</label><input id="kp" class="input" type="number" step="0.1" /><button class="btn" onclick="setParam('Kp', kp.value)">Set</button></div>
        <div class="kv"><label>Ki</label><input id="ki" class="input" type="number" step="0.1" /><button class="btn" onclick="setParam('Ki', ki.value)">Set</button></div>
        <div class="kv"><label>Kd</label><input id="kd" class="input" type="number" step="0.1" /><button class="btn" onclick="setParam('Kd', kd.value)">Set</button></div>
      </div>

      <div class="card">
        <h3>Speed</h3>
        <div class="kv"><label>Base Speed</label><input id="speed" class="input" type="number" step="1" /><button class="btn" onclick="setParam('speed', speed.value)">Set</button></div>
      </div>

      <div class="card">
        <h3>Status</h3>
        <div class="kv">Line Pos: <span id="linePos">0</span></div>
        <div class="kv">PID Out: <span id="pidOut">0.00</span></div>
        <div class="kv">State: <span id="state">-</span></div>
      </div>
    </div>
  </div>

<script>
  function send(cmd){
    fetch('/api/command', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({command:cmd})});
  }
  function setParam(key, val){
    if(!val) return;
    if(key==='speed') send('speed='+val); else send(key+'='+val);
  }
  // Joystick handling
  const joy = document.getElementById('joy');
  const stick = document.getElementById('stick');
  const radius = 100; // movement radius
  let dragging = false;
  function setStick(x,y){ stick.style.transform = `translate(${x}px, ${y}px)`; }
  function sendDrive(){
    // map joyX, joyY (-100..100) to motor speeds -255..255
    // arcade drive: forward = Y, turn = X
    const forward = joyY / 100.0;
    const turn = joyX / 100.0;
    let left = forward + turn;
    let right = forward - turn;
    // normalize to [-1,1]
    const maxv = Math.max(1, Math.max(Math.abs(left), Math.abs(right)));
    left /= maxv; right /= maxv;
    const maxSpeed = 255;
    const lm = Math.round(left * maxSpeed);
    const rm = Math.round(right * maxSpeed);
    fetch('/api/command', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({command:`LM=${lm}`})});
    fetch('/api/command', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({command:`RM=${rm}`})});
  }
  function updateFromPointer(clientX, clientY){
    const rect = joy.getBoundingClientRect();
    const cx = rect.left + rect.width/2;
    const cy = rect.top + rect.height/2;
    let dx = clientX - cx;
    let dy = clientY - cy;
    // clamp to circle radius
    const dist = Math.hypot(dx, dy);
    if (dist > radius) { dx = dx * radius / dist; dy = dy * radius / dist; }
    setStick(dx, dy);
    joyX = Math.round((dx / radius) * 100);
    joyY = Math.round((-dy / radius) * 100);
  }
  function resetStick(){ setStick(0,0); joyX = 0; joyY = 0; send('stopdc'); }
  joy.addEventListener('pointerdown', e=>{ dragging=true; joy.setPointerCapture(e.pointerId); updateFromPointer(e.clientX, e.clientY); });
  joy.addEventListener('pointermove', e=>{ if (!dragging) return; updateFromPointer(e.clientX, e.clientY); });
  joy.addEventListener('pointerup', e=>{ dragging=false; resetStick(); });
  joy.addEventListener('pointercancel', e=>{ dragging=false; resetStick(); });
  // drive loop ~20Hz
  setInterval(()=>{ if (dragging) sendDrive(); }, 50);

  function refresh(){
    fetch('/api/status').then(r=>r.json()).then(d=>{
      document.getElementById('linePos').textContent = d.linePosition;
      document.getElementById('pidOut').textContent = d.pidOutput.toFixed(2);
      document.getElementById('state').textContent = d.state;
      document.getElementById('kp').value = d.Kp;
      document.getElementById('ki').value = d.Ki;
      document.getElementById('kd').value = d.Kd;
      document.getElementById('speed').value = d.baseSpeed;
    });
  }
  setInterval(refresh, 1000);
  refresh();
</script>
</body>
</html>
)html";
    server.send(200, "text/html", html);
  });

  server.on("/api/command", HTTP_POST, [](){
    if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"ok\":false}"); return; }
    DynamicJsonDocument doc(256);
    deserializeJson(doc, server.arg("plain"));
    String cmd = doc["command"] | "";
    if (cmd.length() == 0) { server.send(400, "application/json", "{\"ok\":false}"); return; }

    // Persist only PID and base speed
    if (cmd.startsWith("Kp=") || cmd.startsWith("Ki=") || cmd.startsWith("Kd=") || cmd.startsWith("speed=")) {
      if (cmd.startsWith("Kp=")) gKp = cmd.substring(3).toFloat();
      else if (cmd.startsWith("Ki=")) gKi = cmd.substring(3).toFloat();
      else if (cmd.startsWith("Kd=")) gKd = cmd.substring(3).toFloat();
      else if (cmd.startsWith("speed=")) gBaseSpeed = cmd.substring(6).toInt();
      saveSettings();
    }

    sendCommand(cmd);
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/status", HTTP_GET, [](){
    DynamicJsonDocument doc(256);
    doc["Kp"] = gKp;
    doc["Ki"] = gKi;
    doc["Kd"] = gKd;
    doc["baseSpeed"] = gBaseSpeed;
    doc["linePosition"] = gLinePosition;
    doc["pidOutput"] = gPidOutput;
    doc["state"] = "-";
    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
  });

  server.onNotFound([](){ server.send(404, "text/plain", "Not found"); });
}

void sendCommand(const String& cmd) { Serial.println(cmd); }

void parseSTM32Data(const String& data) {
  if (data.startsWith("State:")) {
    int posKey = data.indexOf("Line Pos:");
    if (posKey > 0) {
      int valStart = posKey + 9; while (valStart < (int)data.length() && data[valStart] == ' ') valStart++;
      int valEnd = data.indexOf(' ', valStart); if (valEnd < 0) valEnd = data.length();
      gLinePosition = data.substring(valStart, valEnd).toInt();
    }
    int pidKey = data.indexOf("PID:");
    if (pidKey > 0) {
      int valStart = pidKey + 4; while (valStart < (int)data.length() && data[valStart] == ' ') valStart++;
      int valEnd = data.indexOf(' ', valStart); if (valEnd < 0) valEnd = data.length();
      gPidOutput = data.substring(valStart, valEnd).toFloat();
    }
  } else if (data.startsWith("PID Values")) { parsePIDData(data); }
}

void parsePIDData(const String& data) {
  int kpKey = data.indexOf("Kp:");
  int kiKey = data.indexOf("Ki:");
  int kdKey = data.indexOf("Kd:");
  int spKey = data.indexOf("Speed:");
  if (kpKey >= 0) gKp = data.substring(kpKey + 3, data.indexOf(' ', kpKey + 3)).toFloat();
  if (kiKey >= 0) gKi = data.substring(kiKey + 3, data.indexOf(' ', kiKey + 3)).toFloat();
  if (kdKey >= 0) gKd = data.substring(kdKey + 3, data.indexOf(' ', kdKey + 3)).toFloat();
  if (spKey >= 0) gBaseSpeed = data.substring(spKey + 6).toInt();
}

void saveSettings() {
  Settings s; s.Kp = gKp; s.Ki = gKi; s.Kd = gKd; s.baseSpeed = gBaseSpeed; s.valid = true;
  EEPROM.put(SETTINGS_ADDRESS, s); EEPROM.commit();
}

void loadSettings() {
  Settings s; EEPROM.get(SETTINGS_ADDRESS, s);
  if (s.valid) { gKp = s.Kp; gKi = s.Ki; gKd = s.Kd; gBaseSpeed = s.baseSpeed; } else { saveSettings(); }
}

void sendSettingsToSTM32() {
  sendCommand("Kp=" + String(gKp, 2)); delay(60);
  sendCommand("Ki=" + String(gKi, 2)); delay(60);
  sendCommand("Kd=" + String(gKd, 2)); delay(60);
  sendCommand("speed=" + String(gBaseSpeed)); delay(60);
}
