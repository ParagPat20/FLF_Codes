/*
   ESP8266 Web Server for STM32 Line Follower Robot Control
   
   Features:
   - Creates its own WiFi Access Point
   - Web interface for remote control
   - EEPROM storage for persistent settings
   - Sends saved settings to STM32 on boot
   - Real-time sensor data display
   - PID tuning interface
   
   Connect ESP8266 to STM32 via Serial (TX/RX)
   Access web interface at: http://192.168.4.1
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

// Access Point Configuration
const char* ap_ssid = "FIBEE";     // AP name
const char* ap_password = "oxitech@1";           // AP password (min 8 chars)
IPAddress local_IP(192, 168, 4, 1);            // AP IP address
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// EEPROM Settings Structure
struct Settings {
  float Kp;
  float Ki; 
  float Kd;
  int baseSpeed;
  float baseThrust;
  
  // Setup Variables
  bool followBlackLine;     // true = follow black, false = follow white
  bool leftMotorInverted;   // true = invert left DC motor direction
  bool rightMotorInverted;  // true = invert right DC motor direction
  bool escSwapped;          // true = ESC1 acts as right, ESC2 as left
  bool esc1Inverted;        // true = invert ESC1 direction
  bool esc2Inverted;        // true = invert ESC2 direction
  bool autoStartLineFollow; // true = start line following on boot
  
  bool valid;  // To check if EEPROM has valid data
};

#define EEPROM_SIZE 512
#define SETTINGS_ADDRESS 0

// Pin definitions
#define LED_PIN 2

// Serial communication with STM32
#define SERIAL_BAUD 115200

// Web server
ESP8266WebServer server(80);

// Robot status variables
bool lineFollowing = false;
bool sweeping = false;
float Kp = 0.15;
float Ki = 0.2;
float Kd = 0.8;
int baseSpeed = 200;
float baseThrust = 0.4;
int sensorValues[7] = {0, 0, 0, 0, 0, 0, 0};
int linePosition = 0;
float pidOutput = 0;

// Setup variables
bool followBlackLine = true;
bool leftMotorInverted = false;
bool rightMotorInverted = false;
bool escSwapped = false;
bool esc1Inverted = false;
bool esc2Inverted = false;
bool autoStartLineFollow = false;

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load settings from EEPROM
  loadSettings();
  
  // Setup Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ap_ssid, ap_password);
  
  digitalWrite(LED_PIN, HIGH);
  
  // Wait a bit for AP to start
  delay(2000);
  
  // Start mDNS
  if (MDNS.begin("linefollower")) {
    // MDNS started
  }
  
  // Setup web server routes
  setupWebServer();
  
  // Start web server
  server.begin();
  
  // Send saved settings to STM32 on boot
  delay(1000);  // Wait for STM32 to be ready
  sendSettingsToSTM32();
  sendSetupToSTM32();
  
  // Auto-start line following if enabled
  if (autoStartLineFollow) {
    delay(500);  // Give STM32 time to process setup
    sendCommand("linefollow");
    lineFollowing = true;
  }
}

void loop() {
  server.handleClient();
  MDNS.update();
  
  // Handle serial communication with STM32
  handleSerialCommunication();
  
  // Request status update periodically
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 1000) {
    lastUpdate = millis();
    sendCommand("pid");
  }
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleGetStatus);
  server.on("/api/command", HTTP_POST, handleCommand);
  server.onNotFound(handleNotFound);
}

void handleRoot() {
  String html = R"html(
<!DOCTYPE html>
<html>
<head>
    <title>Line Follower Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
        .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }
        .status { background: #e8f5e8; padding: 15px; border-radius: 5px; margin: 10px 0; }
        .control { background: #f0f8ff; padding: 15px; border-radius: 5px; margin: 10px 0; }
        .btn { background: #007bff; color: white; padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; margin: 5px; }
        .btn:hover { background: #0056b3; }
        .btn-danger { background: #dc3545; }
        .btn-danger:hover { background: #c82333; }
        .input-group { margin: 10px 0; }
        .input-group label { display: inline-block; width: 100px; }
        .input-group input { padding: 5px; border: 1px solid #ddd; border-radius: 3px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 Line Follower Robot Control</h1>
        
        <div class="status">
            <h3>Connection Info</h3>
            <p><strong>WiFi Network:</strong> FIBEE</p>
            <p><strong>Password:</strong> oxitech@1</p>
            <p><strong>Web Address:</strong> http://192.168.4.1</p>
        </div>
        
        <div class="status">
            <h3>Robot Status</h3>
            <p>Line Following: <span id="lineFollowingStatus">Unknown</span></p>
            <p>Sweeping: <span id="sweepingStatus">Unknown</span></p>
            <p>Line Position: <span id="linePosition">0</span></p>
            <p>PID Output: <span id="pidOutput">0</span></p>
        </div>
        
        <div class="control">
            <h3>Line Following Control</h3>
            <button onclick="sendCommand('linefollow')" class="btn">Start Line Following</button>
            <button onclick="sendCommand('stopline')" class="btn btn-danger">Stop Line Following</button>
            <button onclick="sendCommand('start')" class="btn">Start Sweep</button>
            <button onclick="sendCommand('stop')" class="btn btn-danger">Stop All</button>
        </div>
        
        <div class="control">
            <h3>PID Tuning</h3>
            <div class="input-group">
                <label>Kp:</label>
                <input type="number" id="kpInput" step="0.01" value="0.15">
                <button onclick="updatePID('Kp')" class="btn">Set</button>
            </div>
            <div class="input-group">
                <label>Ki:</label>
                <input type="number" id="kiInput" step="0.01" value="0.2">
                <button onclick="updatePID('Ki')" class="btn">Set</button>
            </div>
            <div class="input-group">
                <label>Kd:</label>
                <input type="number" id="kdInput" step="0.01" value="0.8">
                <button onclick="updatePID('Kd')" class="btn">Set</button>
            </div>
            <div class="input-group">
                <label>Base Speed:</label>
                <input type="number" id="baseSpeedInput" step="10" value="200">
                <button onclick="updatePID('basespeed')" class="btn">Set</button>
            </div>
            <div class="input-group">
                <label>Base Thrust:</label>
                <input type="number" id="baseThrustInput" step="0.1" value="0.4">
                <button onclick="updatePID('thrust')" class="btn">Set</button>
            </div>
        </div>
        
        <div class="control">
            <h3>Setup Configuration</h3>
            <div class="input-group">
                <label>Line Color:</label>
                <select id="lineColorSelect" onchange="updateSetup('linecolor')">
                    <option value="black">Follow Black Line</option>
                    <option value="white">Follow White Line</option>
                </select>
            </div>
            <div class="input-group">
                <label>Left Motor:</label>
                <select id="leftMotorSelect" onchange="updateSetup('leftmotor')">
                    <option value="normal">Normal Direction</option>
                    <option value="inverted">Inverted Direction</option>
                </select>
            </div>
            <div class="input-group">
                <label>Right Motor:</label>
                <select id="rightMotorSelect" onchange="updateSetup('rightmotor')">
                    <option value="normal">Normal Direction</option>
                    <option value="inverted">Inverted Direction</option>
                </select>
            </div>
            <div class="input-group">
                <label>ESC Mapping:</label>
                <select id="escMappingSelect" onchange="updateSetup('escmapping')">
                    <option value="normal">ESC1=Left, ESC2=Right</option>
                    <option value="swapped">ESC1=Right, ESC2=Left</option>
                </select>
            </div>
            <div class="input-group">
                <label>ESC1 Direction:</label>
                <select id="esc1Select" onchange="updateSetup('esc1')">
                    <option value="normal">Normal Direction</option>
                    <option value="inverted">Inverted Direction</option>
                </select>
            </div>
            <div class="input-group">
                <label>ESC2 Direction:</label>
                <select id="esc2Select" onchange="updateSetup('esc2')">
                    <option value="normal">Normal Direction</option>
                    <option value="inverted">Inverted Direction</option>
                </select>
            </div>
            <div class="input-group">
                <label>Auto-Start:</label>
                <select id="autoStartSelect" onchange="updateSetup('autostart')">
                    <option value="disabled">Manual Start</option>
                    <option value="enabled">Auto-Start Line Following</option>
                </select>
            </div>
        </div>
        
        <div class="control">
            <h3>Motor Control</h3>
            <button onclick="sendCommand('stopdc')" class="btn btn-danger">Stop DC Motors</button>
        </div>
    </div>
    
    <script>
        function sendCommand(command) {
            fetch('/api/command', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ command: command })
            })
            .then(response => response.json())
            .then(data => console.log('Command sent:', command))
            .catch(error => console.error('Error:', error));
        }
        
        function updatePID(type) {
            let value, command;
            switch(type) {
                case 'Kp': value = document.getElementById('kpInput').value; command = 'Kp=' + value; break;
                case 'Ki': value = document.getElementById('kiInput').value; command = 'Ki=' + value; break;
                case 'Kd': value = document.getElementById('kdInput').value; command = 'Kd=' + value; break;
                case 'basespeed': value = document.getElementById('baseSpeedInput').value; command = 'basespeed=' + value; break;
                case 'thrust': value = document.getElementById('baseThrustInput').value; command = 'thrust=' + value; break;
            }
            sendCommand(command);
        }
        
        function updateSetup(type) {
            let value, command;
            switch(type) {
                case 'linecolor': 
                    value = document.getElementById('lineColorSelect').value; 
                    command = 'setup_linecolor=' + value; 
                    break;
                case 'leftmotor': 
                    value = document.getElementById('leftMotorSelect').value; 
                    command = 'setup_leftmotor=' + value; 
                    break;
                case 'rightmotor': 
                    value = document.getElementById('rightMotorSelect').value; 
                    command = 'setup_rightmotor=' + value; 
                    break;
                case 'escmapping': 
                    value = document.getElementById('escMappingSelect').value; 
                    command = 'setup_escmapping=' + value; 
                    break;
                case 'esc1': 
                    value = document.getElementById('esc1Select').value; 
                    command = 'setup_esc1=' + value; 
                    break;
                case 'esc2': 
                    value = document.getElementById('esc2Select').value; 
                    command = 'setup_esc2=' + value; 
                    break;
                case 'autostart': 
                    value = document.getElementById('autoStartSelect').value; 
                    command = 'setup_autostart=' + value; 
                    break;
            }
            sendCommand(command);
        }
        
        // Update status every second
        setInterval(() => {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('lineFollowingStatus').textContent = data.lineFollowing ? 'Active' : 'Inactive';
                    document.getElementById('sweepingStatus').textContent = data.sweeping ? 'Active' : 'Inactive';
                    document.getElementById('linePosition').textContent = data.linePosition;
                    document.getElementById('pidOutput').textContent = data.pidOutput.toFixed(2);
                    
                    // Update setup values
                    document.getElementById('lineColorSelect').value = data.followBlackLine ? 'black' : 'white';
                    document.getElementById('leftMotorSelect').value = data.leftMotorInverted ? 'inverted' : 'normal';
                    document.getElementById('rightMotorSelect').value = data.rightMotorInverted ? 'inverted' : 'normal';
                    document.getElementById('escMappingSelect').value = data.escSwapped ? 'swapped' : 'normal';
                    document.getElementById('esc1Select').value = data.esc1Inverted ? 'inverted' : 'normal';
                    document.getElementById('esc2Select').value = data.esc2Inverted ? 'inverted' : 'normal';
                    document.getElementById('autoStartSelect').value = data.autoStartLineFollow ? 'enabled' : 'disabled';
                })
                .catch(error => console.error('Error updating status:', error));
        }, 1000);
    </script>
</body>
</html>
)html";
  
  server.send(200, "text/html", html);
}

void handleGetStatus() {
  DynamicJsonDocument doc(1024);
  doc["lineFollowing"] = lineFollowing;
  doc["sweeping"] = sweeping;
  doc["Kp"] = Kp;
  doc["Ki"] = Ki;
  doc["Kd"] = Kd;
  doc["baseSpeed"] = baseSpeed;
  doc["baseThrust"] = baseThrust;
  doc["linePosition"] = linePosition;
  doc["pidOutput"] = pidOutput;
  
  // Setup variables
  doc["followBlackLine"] = followBlackLine;
  doc["leftMotorInverted"] = leftMotorInverted;
  doc["rightMotorInverted"] = rightMotorInverted;
  doc["escSwapped"] = escSwapped;
  doc["esc1Inverted"] = esc1Inverted;
  doc["esc2Inverted"] = esc2Inverted;
  doc["autoStartLineFollow"] = autoStartLineFollow;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleCommand() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    DynamicJsonDocument doc(256);
    deserializeJson(doc, body);
    
    String command = doc["command"];
    if (command.length() > 0) {
      sendCommand(command);
      
      // Handle setup commands
      if (command.startsWith("setup_")) {
        handleSetupCommand(command);
        saveSettings();
      }
      // Save PID settings to EEPROM when they change
      else if (command.startsWith("Kp=") || command.startsWith("Ki=") || 
               command.startsWith("Kd=") || command.startsWith("basespeed=") || 
               command.startsWith("thrust=")) {
        saveSettings();
      }
      
      server.send(200, "application/json", "{\"success\":true}");
    } else {
      server.send(400, "application/json", "{\"success\":false}");
    }
  } else {
    server.send(400, "application/json", "{\"success\":false}");
  }
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

void sendCommand(String command) {
  Serial.println(command);
}

void handleSerialCommunication() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    parseSTM32Data(data);
  }
}

void parseSTM32Data(String data) {
  if (data.startsWith("Line Pos:")) {
    parseLineFollowingData(data);
  } else if (data.startsWith("Kp:")) {
    parsePIDData(data);
  }
}

void parseLineFollowingData(String data) {
  // Parse line position
  int posStart = data.indexOf("Line Pos: ") + 10;
  int posEnd = data.indexOf(" ", posStart);
  if (posStart > 9 && posEnd > posStart) {
    linePosition = data.substring(posStart, posEnd).toInt();
  }
  
  // Parse PID output
  int pidStart = data.indexOf("PID: ") + 5;
  int pidEnd = data.indexOf(" ", pidStart);
  if (pidStart > 4 && pidEnd > pidStart) {
    pidOutput = data.substring(pidStart, pidEnd).toFloat();
  }
}

void parsePIDData(String data) {
  // Parse Kp
  int kpStart = data.indexOf("Kp: ") + 4;
  int kpEnd = data.indexOf(" ", kpStart);
  if (kpStart > 3 && kpEnd > kpStart) {
    Kp = data.substring(kpStart, kpEnd).toFloat();
  }
  
  // Parse Ki
  int kiStart = data.indexOf("Ki: ") + 4;
  int kiEnd = data.indexOf(" ", kiStart);
  if (kiStart > 3 && kiEnd > kiStart) {
    Ki = data.substring(kiStart, kiEnd).toFloat();
  }
  
  // Parse Kd
  int kdStart = data.indexOf("Kd: ") + 4;
  int kdEnd = data.indexOf(" ", kdStart);
  if (kdStart > 3 && kdEnd > kdStart) {
    Kd = data.substring(kdStart, kdEnd).toFloat();
  }
  
  // Parse base speed
  int speedStart = data.indexOf("Base Speed: ") + 12;
  if (speedStart > 11) {
    baseSpeed = data.substring(speedStart).toInt();
  }
}

// ===== EEPROM FUNCTIONS =====
void saveSettings() {
  Settings settings;
  settings.Kp = Kp;
  settings.Ki = Ki;
  settings.Kd = Kd;
  settings.baseSpeed = baseSpeed;
  settings.baseThrust = baseThrust;
  
  // Setup variables
  settings.followBlackLine = followBlackLine;
  settings.leftMotorInverted = leftMotorInverted;
  settings.rightMotorInverted = rightMotorInverted;
  settings.escSwapped = escSwapped;
  settings.esc1Inverted = esc1Inverted;
  settings.esc2Inverted = esc2Inverted;
  settings.autoStartLineFollow = autoStartLineFollow;
  
  settings.valid = true;
  
  // Write settings to EEPROM
  EEPROM.put(SETTINGS_ADDRESS, settings);
  EEPROM.commit();
}

void loadSettings() {
  Settings settings;
  EEPROM.get(SETTINGS_ADDRESS, settings);
  
  // Check if EEPROM has valid data
  if (settings.valid) {
    Kp = settings.Kp;
    Ki = settings.Ki;
    Kd = settings.Kd;
    baseSpeed = settings.baseSpeed;
    baseThrust = settings.baseThrust;
    
    // Setup variables
    followBlackLine = settings.followBlackLine;
    leftMotorInverted = settings.leftMotorInverted;
    rightMotorInverted = settings.rightMotorInverted;
    escSwapped = settings.escSwapped;
    esc1Inverted = settings.esc1Inverted;
    esc2Inverted = settings.esc2Inverted;
    autoStartLineFollow = settings.autoStartLineFollow;
  } else {
    // Use default values and save them
    Kp = 0.15;
    Ki = 0.2;
    Kd = 0.8;
    baseSpeed = 200;
    baseThrust = 0.4;
    
    // Default setup values
    followBlackLine = true;
    leftMotorInverted = false;
    rightMotorInverted = false;
    escSwapped = false;
    esc1Inverted = false;
    esc2Inverted = false;
    autoStartLineFollow = false;
    
    saveSettings();
  }
}

void sendSettingsToSTM32() {
  // Send all saved settings to STM32 on boot
  sendCommand("Kp=" + String(Kp, 3));
  delay(50);
  sendCommand("Ki=" + String(Ki, 3));
  delay(50);
  sendCommand("Kd=" + String(Kd, 3));
  delay(50);
  sendCommand("basespeed=" + String(baseSpeed));
  delay(50);
  sendCommand("thrust=" + String(baseThrust, 2));
  delay(50);
}

void sendSetupToSTM32() {
  // Send all setup variables to STM32
  sendCommand("setup_linecolor=" + String(followBlackLine ? "black" : "white"));
  delay(50);
  sendCommand("setup_leftmotor=" + String(leftMotorInverted ? "inverted" : "normal"));
  delay(50);
  sendCommand("setup_rightmotor=" + String(rightMotorInverted ? "inverted" : "normal"));
  delay(50);
  sendCommand("setup_escmapping=" + String(escSwapped ? "swapped" : "normal"));
  delay(50);
  sendCommand("setup_esc1=" + String(esc1Inverted ? "inverted" : "normal"));
  delay(50);
  sendCommand("setup_esc2=" + String(esc2Inverted ? "inverted" : "normal"));
  delay(50);
}

void handleSetupCommand(String command) {
  if (command.startsWith("setup_linecolor=")) {
    String value = command.substring(16);
    followBlackLine = (value == "black");
  }
  else if (command.startsWith("setup_leftmotor=")) {
    String value = command.substring(16);
    leftMotorInverted = (value == "inverted");
  }
  else if (command.startsWith("setup_rightmotor=")) {
    String value = command.substring(17);
    rightMotorInverted = (value == "inverted");
  }
  else if (command.startsWith("setup_escmapping=")) {
    String value = command.substring(17);
    escSwapped = (value == "swapped");
  }
  else if (command.startsWith("setup_esc1=")) {
    String value = command.substring(11);
    esc1Inverted = (value == "inverted");
  }
  else if (command.startsWith("setup_esc2=")) {
    String value = command.substring(11);
    esc2Inverted = (value == "inverted");
  }
  else if (command.startsWith("setup_autostart=")) {
    String value = command.substring(16);
    autoStartLineFollow = (value == "enabled");
  }
}
