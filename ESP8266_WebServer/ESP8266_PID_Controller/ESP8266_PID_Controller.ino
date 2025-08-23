/*
 * ESP8266 Web Server for STM32 Line Following Robot Parameter Control
 * 
 * This code creates a web server that allows real-time adjustment of:
 * - PID parameters (Kp, Ki, Kd for base and max speeds)
 * - Motor control parameters (speeds, throttle settings)
 * - Sensor thresholds
 * 
 * Communication with STM32 via Serial at 115200 baud
 * Web interface accessible at robot's IP address
 * 
 * Access Point Mode:
 * SSID: Robot_Controller
 * Password: oxitech@1
 * IP: 192.168.4.1
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <Wire.h>
// Use ESP8266-compatible MPU6050 library
// Install: "MPU6050 by Electronic Cats" or "MPU6050_tockn"
#include <MPU6050_tockn.h>

// Access Point credentials
const char* ap_ssid = "FLF";
const char* ap_password = "oxitech@1";

// Web server on port 80
ESP8266WebServer server(80);

// MPU6050 instance (ESP8266-compatible)
MPU6050 mpu(Wire);

// Robot parameters structure (matches STM32)
struct RobotParameters {
  float baseKp = 0.1;
  float baseKi = 0.02;
  float baseKd = 0.05;
  float maxKp = 0.1;
  float maxKi = 0.02;
  float maxKd = 0.05;
  int baseSpeed = 220;
  int minThrottle = 220;
  int maxThrottle = 250;
  int throttleIncrement = 10;
  unsigned long throttleInterval = 200;
  int maxCorrection = 400;
  int errorDeadband = 10;
  int sensorThresholds[7] = {2550, 2880, 3000, 3500, 3150, 3125, 2600};
} robotParams;

// Current robot status (received from STM32)
struct RobotStatus {
  int currentThrottle = 220;
  bool lineDetected = false;
  float currentPosition = 3000.0;
  float error = 0.0;
  bool inRecoveryMode = false;
  unsigned long lastUpdate = 0;
} robotStatus;

// EEPROM addresses for parameter storage
#define EEPROM_SIZE 512
#define PARAMS_ADDRESS 0

// Timing variables
unsigned long lastSerialSend = 0;
unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_TIMEOUT = 3000;  // Consider offline after 3 seconds

// Parameter update tracking
bool parametersChanged = false;
bool robotStateChanged = false;
bool robotRunning = true;  // Default state is START

// ===== DEAD RECKONING VARIABLES =====
struct Position {
  float x = 0.0;      // X position in cm
  float y = 0.0;      // Y position in cm
  float yaw = 0.0;    // Yaw angle in degrees
  unsigned long timestamp = 0;
};

struct IMUData {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float temperature;
};

// Current robot pose
Position currentPose;
Position lastGoodCheckpoint;

// Calibration offsets are handled automatically by MPU6050_tockn library

// Dead reckoning parameters
float velocityX = 0.0, velocityY = 0.0;  // Current velocity in cm/s
unsigned long lastIMUUpdate = 0;
const unsigned long IMU_UPDATE_INTERVAL = 10;  // 100Hz update rate

// Checkpoint management
bool hasValidCheckpoint = false;
unsigned long lastCheckpointTime = 0;
const unsigned long CHECKPOINT_INTERVAL = 500;  // Save checkpoint every 500ms when on line

// Navigation state
enum NavigationState {
  NAV_FOLLOWING_LINE,
  NAV_RETURNING_TO_CHECKPOINT,
  NAV_SEARCHING
};
NavigationState navState = NAV_FOLLOWING_LINE;

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  
  // Initialize I2C for MPU6050
  Wire.begin();
  
  // Initialize MPU6050
  initializeMPU6050();
  
  // Load parameters from EEPROM
  loadParameters();
  
  // Configure Access Point
  WiFi.mode(WIFI_AP);
  
  Serial.println();
  Serial.println("Configuring Access Point...");
  
  // Configure AP with specific settings for better compatibility
  bool result = WiFi.softAP(ap_ssid, ap_password, 1, 0, 4); // channel 1, hidden=false, max_connections=4
  
  if(result) {
    Serial.println("Access Point Started Successfully!");
    Serial.print("SSID: ");
    Serial.println(ap_ssid);
    Serial.print("Password: ");
    Serial.println(ap_password);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("MAC Address: ");
    Serial.println(WiFi.softAPmacAddress());
    Serial.println("Connect to this network and go to http://192.168.4.1");
  } else {
    Serial.println("Failed to start Access Point!");
    Serial.println("Restarting ESP8266...");
    delay(3000);
    ESP.restart();
  }
  
  // Setup web server routes
  setupWebServer();
  
  // Start server
  server.begin();
  Serial.println("Web server started successfully");
  Serial.print("Free heap: ");
  Serial.println(ESP.getFreeHeap());
  
  // Send initial parameters to STM32
  sendParametersToSTM32();
}

// ===== MPU6050 AND DEAD RECKONING FUNCTIONS =====
void initializeMPU6050() {
  Serial.println("Initializing MPU6050...");
  
  // Initialize MPU6050
  mpu.begin();
  
  // Test connection by trying to read data
  mpu.update();
  
  // Check if we can get valid temperature reading (indicates successful connection)
  float testTemp = mpu.getTemp();
  
  if (!isnan(testTemp) && testTemp > -40 && testTemp < 85) {
    Serial.println("MPU6050 connection successful");
    Serial.printf("Temperature: %.2f°C\n", testTemp);
    
    // Calibrate IMU (built-in function)
    Serial.println("Calibrating gyroscope... Keep robot stationary!");
    mpu.calcGyroOffsets(true);
    Serial.println("MPU6050 calibration complete!");
    
    // Initialize pose
    currentPose.x = 0.0;
    currentPose.y = 0.0;
    currentPose.yaw = 0.0;
    currentPose.timestamp = millis();
    
    lastGoodCheckpoint = currentPose;
    
  } else {
    Serial.println("MPU6050 connection failed!");
    Serial.printf("Invalid temperature reading: %.2f\n", testTemp);
  }
}

// Calibration is now handled automatically by the MPU6050_tockn library

IMUData readIMU() {
  IMUData data;
  
  // Update MPU6050 readings
  mpu.update();
  
  // Get calibrated values directly from the library
  data.accelX = mpu.getAccX() * 9.81; // Convert to m/s²
  data.accelY = mpu.getAccY() * 9.81;
  data.accelZ = mpu.getAccZ() * 9.81;
  
  data.gyroX = mpu.getGyroX(); // Already in °/s
  data.gyroY = mpu.getGyroY();
  data.gyroZ = mpu.getGyroZ();
  
  data.temperature = mpu.getTemp();
  
  return data;
}

void updateDeadReckoning() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastIMUUpdate >= IMU_UPDATE_INTERVAL) {
    IMUData imuData = readIMU();
    float dt = (currentTime - lastIMUUpdate) / 1000.0; // Convert to seconds
    
    // Update yaw using gyroscope (most important for line following)
    currentPose.yaw += imuData.gyroZ * dt;
    
    // Keep yaw in -180 to +180 range
    while (currentPose.yaw > 180.0) currentPose.yaw -= 360.0;
    while (currentPose.yaw < -180.0) currentPose.yaw += 360.0;
    
    // Estimate velocity from robot's current throttle (from STM32 status)
    // Assume linear relationship: throttle 220-250 maps to 0-30 cm/s
    float estimatedSpeed = 0.0;
    if (robotStatus.currentThrottle > 0) {
      estimatedSpeed = map(robotStatus.currentThrottle, 220, 250, 0, 30); // cm/s
    }
    
    // Calculate velocity components based on current yaw
    float yawRad = currentPose.yaw * PI / 180.0;
    velocityX = estimatedSpeed * cos(yawRad);
    velocityY = estimatedSpeed * sin(yawRad);
    
    // Update position using dead reckoning
    currentPose.x += velocityX * dt;
    currentPose.y += velocityY * dt;
    currentPose.timestamp = currentTime;
    
    lastIMUUpdate = currentTime;
  }
}

void updateCheckpoints() {
  unsigned long currentTime = millis();
  
  // Save checkpoint when robot is on line and following successfully
  if (robotStatus.lineDetected && !robotStatus.inRecoveryMode && 
      (currentTime - lastCheckpointTime) >= CHECKPOINT_INTERVAL) {
    
    lastGoodCheckpoint = currentPose;
    hasValidCheckpoint = true;
    lastCheckpointTime = currentTime;
    navState = NAV_FOLLOWING_LINE;
    
    Serial.printf("Checkpoint saved: X=%.2f, Y=%.2f, Yaw=%.2f\n", 
                  lastGoodCheckpoint.x, lastGoodCheckpoint.y, lastGoodCheckpoint.yaw);
  }
  
  // Update navigation state based on line detection
  if (!robotStatus.lineDetected && robotStatus.inRecoveryMode && hasValidCheckpoint) {
    if (navState == NAV_FOLLOWING_LINE) {
      navState = NAV_RETURNING_TO_CHECKPOINT;
      Serial.println("Line lost - switching to return to checkpoint mode");
    }
  }
}

float calculateDistanceToCheckpoint() {
  if (!hasValidCheckpoint) return -1.0;
  
  float dx = lastGoodCheckpoint.x - currentPose.x;
  float dy = lastGoodCheckpoint.y - currentPose.y;
  return sqrt(dx*dx + dy*dy);
}

float calculateAngleToCheckpoint() {
  if (!hasValidCheckpoint) return 0.0;
  
  float dx = lastGoodCheckpoint.x - currentPose.x;
  float dy = lastGoodCheckpoint.y - currentPose.y;
  float targetAngle = atan2(dy, dx) * 180.0 / PI;
  
  // Calculate the angle difference
  float angleDiff = targetAngle - currentPose.yaw;
  
  // Normalize to -180 to +180
  while (angleDiff > 180.0) angleDiff -= 360.0;
  while (angleDiff < -180.0) angleDiff += 360.0;
  
  return angleDiff;
}

void loop() {
  server.handleClient();
  
  // Update dead reckoning
  updateDeadReckoning();
  
  // Update checkpoint system
  updateCheckpoints();
  
  // Read status from STM32
  readSTM32Status();
  
  // Send updates to STM32 only when parameters or state changed
  if (parametersChanged || robotStateChanged) {
    sendParametersToSTM32();
    parametersChanged = false;
    robotStateChanged = false;
    lastSerialSend = millis();
  }
  
  // Debug: Print client connections every 10 seconds
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 10000) {
    Serial.print("Connected clients: ");
    Serial.println(WiFi.softAPgetStationNum());
    Serial.print("Free heap: ");
    Serial.println(ESP.getFreeHeap());
    Serial.printf("Position: X=%.2f, Y=%.2f, Yaw=%.2f\n", currentPose.x, currentPose.y, currentPose.yaw);
    if (hasValidCheckpoint) {
      Serial.printf("Distance to checkpoint: %.2fcm, Angle: %.2f°\n", 
                    calculateDistanceToCheckpoint(), calculateAngleToCheckpoint());
    }
    lastDebugTime = millis();
  }
}

void setupWebServer() {
  // Main page (simple test version)
  server.on("/", HTTP_GET, handleRoot);
  
  // Full interface
  server.on("/full", HTTP_GET, handleFullInterface);
  
  // API endpoints
  server.on("/api/parameters", HTTP_GET, handleGetParameters);
  server.on("/api/parameters", HTTP_POST, handleSetParameters);
  server.on("/api/status", HTTP_GET, handleGetStatus);
  server.on("/api/reset", HTTP_POST, handleReset);
  server.on("/api/save", HTTP_POST, handleSave);
  server.on("/api/load", HTTP_POST, handleLoad);
  
  // Individual parameter updates
  server.on("/api/pid", HTTP_POST, handleUpdatePID);
  server.on("/api/motor", HTTP_POST, handleUpdateMotor);
  server.on("/api/sensors", HTTP_POST, handleUpdateSensors);
  
  // Robot control
  server.on("/api/start", HTTP_POST, handleStart);
  server.on("/api/stop", HTTP_POST, handleStop);
  
  // Navigation endpoints
  server.on("/api/navigation", HTTP_GET, handleGetNavigation);
  server.on("/api/reset_pose", HTTP_POST, handleResetPose);
  server.on("/api/reset_checkpoint", HTTP_POST, handleResetCheckpoint);
  
  // Map visualization
  server.on("/map", HTTP_GET, handleMapVisualization);
  
  server.onNotFound(handleNotFound);
}

void handleRoot() {
  // Test with simple HTML first to diagnose connectivity
  String html = "<!DOCTYPE html><html><head><title>Robot Controller Test</title></head>";
  html += "<body><h1>ESP8266 Web Server Working!</h1>";
  html += "<p>Access Point: " + String(ap_ssid) + "</p>";
  html += "<p>IP: " + WiFi.softAPIP().toString() + "</p>";
  html += "<p>Connected Clients: " + String(WiFi.softAPgetStationNum()) + "</p>";
  html += "<p>Free Heap: " + String(ESP.getFreeHeap()) + " bytes</p>";
  html += "<p><a href='/full'>Load Full Interface</a></p>";
  html += "<p><a href='/map'>View Dead Reckoning Map</a></p>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

void handleFullInterface() {
  // Check available memory before generating large HTML
  if(ESP.getFreeHeap() < 10000) {
    server.send(200, "text/html", 
      "<html><body><h1>Low Memory!</h1><p>Free Heap: " + 
      String(ESP.getFreeHeap()) + 
      " bytes</p><p>Cannot load full interface. Try restarting ESP8266.</p></body></html>");
    return;
  }
  
  // Send HTML in chunks to avoid memory issues
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  
  // Send HTML header
  server.sendContent("<!DOCTYPE html><html><head><meta charset='UTF-8'>");
  server.sendContent("<title>Robot Controller</title>");
  server.sendContent("<style>body{font-family:Arial;margin:20px;background:#f0f0f0}");
  server.sendContent(".container{max-width:800px;margin:0 auto;background:white;padding:20px;border-radius:10px}");
  server.sendContent(".status{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:15px;margin-bottom:20px}");
  server.sendContent(".status-item{text-align:center;padding:10px;background:#e3f2fd;border-radius:5px}");
  server.sendContent(".status-value{font-size:1.5em;font-weight:bold;color:#1976d2}");
  server.sendContent(".control-group{margin:20px 0;padding:15px;border:1px solid #ddd;border-radius:5px}");
  server.sendContent(".btn{padding:10px 20px;margin:5px;border:none;border-radius:5px;cursor:pointer}");
  server.sendContent(".btn-start{background:#4caf50;color:white}.btn-stop{background:#f44336;color:white}");
  server.sendContent(".btn-update{background:#2196f3;color:white}");
  server.sendContent("input[type=number],input[type=range]{width:100%;padding:5px;margin:5px 0}");
  server.sendContent("</style></head><body>");
  
  // Send HTML body
  server.sendContent("<div class='container'>");
  server.sendContent("<h1>🤖 Line Following Robot Controller</h1>");
  
  // Status section
  server.sendContent("<div class='status'>");
  server.sendContent("<div class='status-item'><div class='status-value' id='throttle'>220</div><div>Throttle</div></div>");
  server.sendContent("<div class='status-item'><div class='status-value' id='position'>3000</div><div>Position</div></div>");
  server.sendContent("<div class='status-item'><div class='status-value' id='error'>0.0</div><div>Error</div></div>");
  server.sendContent("<div class='status-item'><button id='startStopBtn' class='btn btn-start' onclick='toggleRobot()'>START</button><div>Control</div></div>");
  server.sendContent("</div>");
  
  // PID Controls
  server.sendContent("<div class='control-group'>");
  server.sendContent("<h3>PID Parameters</h3>");
  server.sendContent("<label>Base Kp: <input type='number' id='baseKp' min='0' max='1' step='0.01' value='0.1' style='width:100px;'></label><br>");
  server.sendContent("<label>Base Ki: <input type='number' id='baseKi' min='0' max='0.1' step='0.001' value='0.02' style='width:100px;'></label><br>");
  server.sendContent("<label>Base Kd: <input type='number' id='baseKd' min='0' max='0.2' step='0.005' value='0.05' style='width:100px;'></label><br>");
  server.sendContent("<label>Max Kp: <input type='number' id='maxKp' min='0' max='1' step='0.01' value='0.1' style='width:100px;'></label><br>");
  server.sendContent("<label>Max Ki: <input type='number' id='maxKi' min='0' max='0.1' step='0.001' value='0.02' style='width:100px;'></label><br>");
  server.sendContent("<label>Max Kd: <input type='number' id='maxKd' min='0' max='0.2' step='0.005' value='0.05' style='width:100px;'></label><br>");
  server.sendContent("<button class='btn btn-update' onclick='updatePID()'>Update PID</button>");
  server.sendContent("</div>");
  
  // Motor Controls
  server.sendContent("<div class='control-group'>");
  server.sendContent("<h3>Motor Parameters</h3>");
  server.sendContent("<label>Base Speed: <input type='number' id='baseSpeed' min='100' max='300' value='220' style='width:100px;'></label><br>");
  server.sendContent("<label>Min Throttle: <input type='number' id='minThrottle' min='100' max='300' value='220' style='width:100px;'></label><br>");
  server.sendContent("<label>Max Throttle: <input type='number' id='maxThrottle' min='200' max='400' value='250' style='width:100px;'></label><br>");
  server.sendContent("<label>Max Correction: <input type='number' id='maxCorrection' min='100' max='800' value='400' style='width:100px;'></label><br>");
  server.sendContent("<label>Error Deadband: <input type='number' id='errorDeadband' min='0' max='50' value='10' style='width:100px;'></label><br>");
  server.sendContent("<label>Throttle Increment: <input type='number' id='throttleIncrement' min='1' max='50' value='10' style='width:100px;'></label><br>");
  server.sendContent("<label>Throttle Interval (ms): <input type='number' id='throttleInterval' min='50' max='1000' step='50' value='200' style='width:100px;'></label><br>");
  server.sendContent("<button class='btn btn-update' onclick='updateMotor()'>Update Motor</button>");
  server.sendContent("</div>");
  
  // JavaScript
  server.sendContent("<script>");
  server.sendContent("function toggleRobot() { ");
  server.sendContent("  const btn = document.getElementById('startStopBtn');");
  server.sendContent("  const isRunning = btn.textContent === 'STOP';");
  server.sendContent("  fetch(isRunning ? '/api/stop' : '/api/start', {method:'POST'})");
  server.sendContent("    .then(r => r.json()).then(d => { btn.textContent = isRunning ? 'START' : 'STOP'; btn.className = 'btn ' + (isRunning ? 'btn-start' : 'btn-stop'); });");
  server.sendContent("}");
  server.sendContent("function updatePID() {");
  server.sendContent("  const data = { baseKp: parseFloat(document.getElementById('baseKp').value), baseKi: parseFloat(document.getElementById('baseKi').value), baseKd: parseFloat(document.getElementById('baseKd').value), maxKp: parseFloat(document.getElementById('maxKp').value), maxKi: parseFloat(document.getElementById('maxKi').value), maxKd: parseFloat(document.getElementById('maxKd').value) };");
  server.sendContent("  fetch('/api/pid', { method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(data) }).then(r => r.json()).then(d => alert(d.status));");
  server.sendContent("}");
  server.sendContent("function updateMotor() {");
  server.sendContent("  const data = { baseSpeed: parseInt(document.getElementById('baseSpeed').value), minThrottle: parseInt(document.getElementById('minThrottle').value), maxThrottle: parseInt(document.getElementById('maxThrottle').value), maxCorrection: parseInt(document.getElementById('maxCorrection').value), errorDeadband: parseInt(document.getElementById('errorDeadband').value), throttleIncrement: parseInt(document.getElementById('throttleIncrement').value), throttleInterval: parseInt(document.getElementById('throttleInterval').value) };");
  server.sendContent("  fetch('/api/motor', { method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(data) }).then(r => r.json()).then(d => alert(d.status));");
  server.sendContent("}");
  server.sendContent("setInterval(() => { fetch('/api/status').then(r => r.json()).then(d => { ");
  server.sendContent("  document.getElementById('throttle').textContent = d.currentThrottle;");
  server.sendContent("  document.getElementById('position').textContent = d.currentPosition.toFixed(1);");
  server.sendContent("  document.getElementById('error').textContent = d.error.toFixed(2);");
  server.sendContent("  const btn = document.getElementById('startStopBtn');");
  server.sendContent("  btn.textContent = d.robotRunning ? 'STOP' : 'START';");
  server.sendContent("  btn.className = 'btn ' + (d.robotRunning ? 'btn-stop' : 'btn-start');");
  server.sendContent("}); }, 1000);");
  server.sendContent("</script>");
  
  server.sendContent("</div></body></html>");
  server.sendContent(""); // End chunked transfer
}

void handleGetParameters() {
  DynamicJsonDocument doc(1024);
  
  doc["baseKp"] = robotParams.baseKp;
  doc["baseKi"] = robotParams.baseKi;
  doc["baseKd"] = robotParams.baseKd;
  doc["maxKp"] = robotParams.maxKp;
  doc["maxKi"] = robotParams.maxKi;
  doc["maxKd"] = robotParams.maxKd;
  doc["baseSpeed"] = robotParams.baseSpeed;
  doc["minThrottle"] = robotParams.minThrottle;
  doc["maxThrottle"] = robotParams.maxThrottle;
  doc["throttleIncrement"] = robotParams.throttleIncrement;
  doc["throttleInterval"] = robotParams.throttleInterval;
  doc["maxCorrection"] = robotParams.maxCorrection;
  doc["errorDeadband"] = robotParams.errorDeadband;
  
  JsonArray thresholds = doc.createNestedArray("sensorThresholds");
  for(int i = 0; i < 7; i++) {
    thresholds.add(robotParams.sensorThresholds[i]);
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleSetParameters() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    
    if (!error) {
      // Update parameters
      if (doc.containsKey("baseKp")) robotParams.baseKp = doc["baseKp"];
      if (doc.containsKey("baseKi")) robotParams.baseKi = doc["baseKi"];
      if (doc.containsKey("baseKd")) robotParams.baseKd = doc["baseKd"];
      if (doc.containsKey("maxKp")) robotParams.maxKp = doc["maxKp"];
      if (doc.containsKey("maxKi")) robotParams.maxKi = doc["maxKi"];
      if (doc.containsKey("maxKd")) robotParams.maxKd = doc["maxKd"];
      if (doc.containsKey("baseSpeed")) robotParams.baseSpeed = doc["baseSpeed"];
      if (doc.containsKey("maxCorrection")) robotParams.maxCorrection = doc["maxCorrection"];
      if (doc.containsKey("errorDeadband")) robotParams.errorDeadband = doc["errorDeadband"];
      if (doc.containsKey("throttleIncrement")) robotParams.throttleIncrement = doc["throttleIncrement"];
      if (doc.containsKey("throttleInterval")) robotParams.throttleInterval = doc["throttleInterval"];
      
      if (doc.containsKey("sensorThresholds")) {
        JsonArray thresholds = doc["sensorThresholds"];
        for(int i = 0; i < 7 && i < thresholds.size(); i++) {
          robotParams.sensorThresholds[i] = thresholds[i];
        }
      }
      
      // Mark parameters as changed for sending to STM32
      parametersChanged = true;
      
      server.send(200, "application/json", "{\"status\":\"success\"}");
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
    }
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No data received\"}");
  }
}

void handleGetStatus() {
  DynamicJsonDocument doc(512);
  
  doc["currentThrottle"] = robotStatus.currentThrottle;
  doc["lineDetected"] = robotStatus.lineDetected;
  doc["currentPosition"] = robotStatus.currentPosition;
  doc["error"] = robotStatus.error;
  doc["inRecoveryMode"] = robotStatus.inRecoveryMode;
  doc["connected"] = (millis() - robotStatus.lastUpdate) < STATUS_TIMEOUT;
  doc["lastUpdate"] = robotStatus.lastUpdate;
  doc["connectedClients"] = WiFi.softAPgetStationNum();
  doc["freeHeap"] = ESP.getFreeHeap();
  doc["accessPointIP"] = WiFi.softAPIP().toString();
  doc["robotRunning"] = robotRunning;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleUpdatePID() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    
    if (!error) {
      if (doc.containsKey("baseKp")) robotParams.baseKp = doc["baseKp"];
      if (doc.containsKey("baseKi")) robotParams.baseKi = doc["baseKi"];
      if (doc.containsKey("baseKd")) robotParams.baseKd = doc["baseKd"];
      if (doc.containsKey("maxKp")) robotParams.maxKp = doc["maxKp"];
      if (doc.containsKey("maxKi")) robotParams.maxKi = doc["maxKi"];
      if (doc.containsKey("maxKd")) robotParams.maxKd = doc["maxKd"];
      
      parametersChanged = true;
      server.send(200, "application/json", "{\"status\":\"PID updated\"}");
    } else {
      server.send(400, "application/json", "{\"status\":\"error\"}");
    }
  }
}

void handleUpdateMotor() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    
    if (!error) {
      if (doc.containsKey("baseSpeed")) robotParams.baseSpeed = doc["baseSpeed"];
      if (doc.containsKey("minThrottle")) robotParams.minThrottle = doc["minThrottle"];
      if (doc.containsKey("maxThrottle")) robotParams.maxThrottle = doc["maxThrottle"];
      if (doc.containsKey("maxCorrection")) robotParams.maxCorrection = doc["maxCorrection"];
      if (doc.containsKey("errorDeadband")) robotParams.errorDeadband = doc["errorDeadband"];
      if (doc.containsKey("throttleIncrement")) robotParams.throttleIncrement = doc["throttleIncrement"];
      if (doc.containsKey("throttleInterval")) robotParams.throttleInterval = doc["throttleInterval"];
      
      parametersChanged = true;
      server.send(200, "application/json", "{\"status\":\"Motor parameters updated\"}");
    } else {
      server.send(400, "application/json", "{\"status\":\"error\"}");
    }
  }
}

void handleUpdateSensors() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    
    if (!error && doc.containsKey("sensorThresholds")) {
      JsonArray thresholds = doc["sensorThresholds"];
      for(int i = 0; i < 7 && i < thresholds.size(); i++) {
        robotParams.sensorThresholds[i] = thresholds[i];
      }
      
      parametersChanged = true;
      server.send(200, "application/json", "{\"status\":\"Sensor thresholds updated\"}");
    } else {
      server.send(400, "application/json", "{\"status\":\"error\"}");
    }
  }
}

void handleReset() {
  // Reset to default values
  robotParams.baseKp = 0.1;
  robotParams.baseKi = 0.02;
  robotParams.baseKd = 0.05;
  robotParams.maxKp = 0.1;
  robotParams.maxKi = 0.02;
  robotParams.maxKd = 0.05;
  robotParams.baseSpeed = 220;
  robotParams.maxCorrection = 400;
  robotParams.errorDeadband = 10;
  robotParams.throttleIncrement = 10;
  robotParams.throttleInterval = 200;
  
  int defaultThresholds[7] = {2550, 2880, 3000, 3500, 3150, 3125, 2600};
  for(int i = 0; i < 7; i++) {
    robotParams.sensorThresholds[i] = defaultThresholds[i];
  }
  
  parametersChanged = true;
  server.send(200, "application/json", "{\"status\":\"Parameters reset to defaults\"}");
}

void handleSave() {
  saveParameters();
  server.send(200, "application/json", "{\"status\":\"Parameters saved to EEPROM\"}");
}

void handleLoad() {
  loadParameters();
  parametersChanged = true;
  server.send(200, "application/json", "{\"status\":\"Parameters loaded from EEPROM\"}");
}

void handleStart() {
  robotRunning = true;
  robotStateChanged = true;
  server.send(200, "application/json", "{\"status\":\"Robot started\"}");
}

void handleStop() {
  robotRunning = false;
  robotStateChanged = true;
  server.send(200, "application/json", "{\"status\":\"Robot stopped\"}");
}

void handleGetNavigation() {
  DynamicJsonDocument doc(512);
  
  // Current pose
  doc["currentX"] = currentPose.x;
  doc["currentY"] = currentPose.y;
  doc["currentYaw"] = currentPose.yaw;
  doc["timestamp"] = currentPose.timestamp;
  
  // Checkpoint information
  doc["hasCheckpoint"] = hasValidCheckpoint;
  if (hasValidCheckpoint) {
    doc["checkpointX"] = lastGoodCheckpoint.x;
    doc["checkpointY"] = lastGoodCheckpoint.y;
    doc["checkpointYaw"] = lastGoodCheckpoint.yaw;
    doc["distanceToCheckpoint"] = calculateDistanceToCheckpoint();
    doc["angleToCheckpoint"] = calculateAngleToCheckpoint();
  }
  
  // Navigation state
  doc["navigationState"] = navState;
  doc["velocityX"] = velocityX;
  doc["velocityY"] = velocityY;
  
  // Navigation commands for STM32
  if (navState == NAV_RETURNING_TO_CHECKPOINT && hasValidCheckpoint) {
    float distance = calculateDistanceToCheckpoint();
    float angle = calculateAngleToCheckpoint();
    
    doc["shouldReturn"] = true;
    doc["returnDistance"] = distance;
    doc["returnAngle"] = angle;
    
    // Simple navigation commands
    if (distance > 5.0) { // More than 5cm away
      if (abs(angle) > 15.0) { // Need to turn
        doc["command"] = (angle > 0) ? "TURN_LEFT" : "TURN_RIGHT";
        doc["intensity"] = min(abs(angle) / 90.0, 1.0); // Normalize turn intensity
      } else { // Go straight back
        doc["command"] = "REVERSE";
        doc["intensity"] = min(distance / 20.0, 1.0); // Normalize reverse intensity
      }
    } else { // Close to checkpoint
      doc["command"] = "ARRIVED";
      doc["intensity"] = 0.0;
      // Reset navigation state
      navState = NAV_SEARCHING;
    }
  } else {
    doc["shouldReturn"] = false;
    doc["command"] = "NONE";
    doc["intensity"] = 0.0;
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleResetPose() {
  currentPose.x = 0.0;
  currentPose.y = 0.0;
  currentPose.yaw = 0.0;
  currentPose.timestamp = millis();
  velocityX = 0.0;
  velocityY = 0.0;
  
  server.send(200, "application/json", "{\"status\":\"Pose reset to origin\"}");
}

void handleResetCheckpoint() {
  hasValidCheckpoint = false;
  navState = NAV_FOLLOWING_LINE;
  
  server.send(200, "application/json", "{\"status\":\"Checkpoint cleared\"}");
}

void handleMapVisualization() {
  // Check available memory before generating large HTML
  if(ESP.getFreeHeap() < 8000) {
    server.send(200, "text/html", 
      "<html><body><h1>Low Memory!</h1><p>Cannot load map. Try restarting ESP8266.</p></body></html>");
    return;
  }
  
  // Send HTML in chunks
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  
  // Send HTML header
  server.sendContent("<!DOCTYPE html><html><head><meta charset='UTF-8'>");
  server.sendContent("<title>Robot Dead Reckoning Map</title>");
  server.sendContent("<style>");
  server.sendContent("body{font-family:Arial;margin:0;background:#f0f0f0}");
  server.sendContent(".container{max-width:1200px;margin:0 auto;padding:20px}");
  server.sendContent(".header{text-align:center;margin-bottom:20px}");
  server.sendContent(".map-container{background:white;border-radius:10px;padding:20px;box-shadow:0 4px 8px rgba(0,0,0,0.1)}");
  server.sendContent(".map-canvas{border:2px solid #333;background:#f9f9f9;display:block;margin:0 auto}");
  server.sendContent(".controls{margin-top:20px;text-align:center}");
  server.sendContent(".btn{padding:10px 20px;margin:5px;border:none;border-radius:5px;cursor:pointer;background:#2196f3;color:white}");
  server.sendContent(".info-panel{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:15px;margin-top:20px}");
  server.sendContent(".info-item{background:#e3f2fd;padding:15px;border-radius:8px;text-align:center}");
  server.sendContent(".info-value{font-size:1.2em;font-weight:bold;color:#1976d2}");
  server.sendContent("</style></head><body>");
  
  server.sendContent("<div class='container'>");
  server.sendContent("<div class='header'>");
  server.sendContent("<h1>🗺️ Robot Dead Reckoning Map</h1>");
  server.sendContent("<p>Real-time visualization of robot position and path</p>");
  server.sendContent("</div>");
  
  server.sendContent("<div class='map-container'>");
  server.sendContent("<canvas id='mapCanvas' class='map-canvas' width='800' height='600'></canvas>");
  server.sendContent("<div class='controls'>");
  server.sendContent("<button class='btn' onclick='resetMap()'>Reset Map</button>");
  server.sendContent("<button class='btn' onclick='centerView()'>Center View</button>");
  server.sendContent("<button class='btn' onclick='togglePath()'>Toggle Path</button>");
  server.sendContent("</div>");
  server.sendContent("</div>");
  
  server.sendContent("<div class='info-panel'>");
  server.sendContent("<div class='info-item'><div class='info-value' id='posX'>0.0</div><div>X Position (cm)</div></div>");
  server.sendContent("<div class='info-item'><div class='info-value' id='posY'>0.0</div><div>Y Position (cm)</div></div>");
  server.sendContent("<div class='info-item'><div class='info-value' id='yaw'>0.0</div><div>Yaw Angle (°)</div></div>");
  server.sendContent("<div class='info-item'><div class='info-value' id='distance'>0.0</div><div>Distance to Checkpoint (cm)</div></div>");
  server.sendContent("</div>");
  
  // JavaScript for map visualization
  server.sendContent("<script>");
  server.sendContent("const canvas = document.getElementById('mapCanvas');");
  server.sendContent("const ctx = canvas.getContext('2d');");
  server.sendContent("let robotPath = [];");
  server.sendContent("let showPath = true;");
  server.sendContent("let scale = 2.0;");
  server.sendContent("let offsetX = 400, offsetY = 300;");
  
  server.sendContent("function drawMap() {");
  server.sendContent("  ctx.clearRect(0, 0, canvas.width, canvas.height);");
  server.sendContent("  ctx.fillStyle = '#f9f9f9';");
  server.sendContent("  ctx.fillRect(0, 0, canvas.width, canvas.height);");
  
  // Draw grid
  server.sendContent("  ctx.strokeStyle = '#e0e0e0';");
  server.sendContent("  ctx.lineWidth = 1;");
  server.sendContent("  for(let x = 0; x < canvas.width; x += 20) {");
  server.sendContent("    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, canvas.height); ctx.stroke();");
  server.sendContent("  }");
  server.sendContent("  for(let y = 0; y < canvas.height; y += 20) {");
  server.sendContent("    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvas.width, y); ctx.stroke();");
  server.sendContent("  }");
  
  // Draw center axes
  server.sendContent("  ctx.strokeStyle = '#666';");
  server.sendContent("  ctx.lineWidth = 2;");
  server.sendContent("  ctx.beginPath(); ctx.moveTo(offsetX, 0); ctx.lineTo(offsetX, canvas.height); ctx.stroke();");
  server.sendContent("  ctx.beginPath(); ctx.moveTo(0, offsetY); ctx.lineTo(canvas.width, offsetY); ctx.stroke();");
  server.sendContent("}");
  
  server.sendContent("function updateMap() {");
  server.sendContent("  fetch('/api/navigation')");
  server.sendContent("    .then(r => r.json())");
  server.sendContent("    .then(data => {");
  server.sendContent("      drawMap();");
  
  // Draw path
  server.sendContent("      if(showPath && robotPath.length > 1) {");
  server.sendContent("        ctx.strokeStyle = '#2196f3';");
  server.sendContent("        ctx.lineWidth = 2;");
  server.sendContent("        ctx.beginPath();");
  server.sendContent("        ctx.moveTo(robotPath[0].x, robotPath[0].y);");
  server.sendContent("        for(let i = 1; i < robotPath.length; i++) {");
  server.sendContent("          ctx.lineTo(robotPath[i].x, robotPath[i].y);");
  server.sendContent("        }");
  server.sendContent("        ctx.stroke();");
  server.sendContent("      }");
  
  // Draw checkpoint
  server.sendContent("      if(data.hasCheckpoint) {");
  server.sendContent("        const cpX = offsetX + data.checkpointX * scale;");
  server.sendContent("        const cpY = offsetY - data.checkpointY * scale;");
  server.sendContent("        ctx.fillStyle = '#4caf50';");
  server.sendContent("        ctx.beginPath();");
  server.sendContent("        ctx.arc(cpX, cpY, 8, 0, 2*Math.PI);");
  server.sendContent("        ctx.fill();");
  server.sendContent("        ctx.fillStyle = '#fff';");
  server.sendContent("        ctx.font = '12px Arial';");
  server.sendContent("        ctx.textAlign = 'center';");
  server.sendContent("        ctx.fillText('CP', cpX, cpY + 4);");
  server.sendContent("      }");
  
  // Draw robot
  server.sendContent("      const robotX = offsetX + data.currentX * scale;");
  server.sendContent("      const robotY = offsetY - data.currentY * scale;");
  server.sendContent("      robotPath.push({x: robotX, y: robotY});");
  server.sendContent("      if(robotPath.length > 1000) robotPath.shift();");
  
  server.sendContent("      ctx.save();");
  server.sendContent("      ctx.translate(robotX, robotY);");
  server.sendContent("      ctx.rotate(-data.currentYaw * Math.PI / 180);");
  server.sendContent("      ctx.fillStyle = '#f44336';");
  server.sendContent("      ctx.fillRect(-10, -6, 20, 12);");
  server.sendContent("      ctx.fillStyle = '#fff';");
  server.sendContent("      ctx.fillRect(6, -4, 4, 8);");
  server.sendContent("      ctx.restore();");
  
  // Update info panel
  server.sendContent("      document.getElementById('posX').textContent = data.currentX.toFixed(1);");
  server.sendContent("      document.getElementById('posY').textContent = data.currentY.toFixed(1);");
  server.sendContent("      document.getElementById('yaw').textContent = data.currentYaw.toFixed(1);");
  server.sendContent("      document.getElementById('distance').textContent = data.hasCheckpoint ? data.distanceToCheckpoint.toFixed(1) : 'N/A';");
  server.sendContent("    });");
  server.sendContent("}");
  
  server.sendContent("function resetMap() { robotPath = []; }");
  server.sendContent("function centerView() { offsetX = 400; offsetY = 300; }");
  server.sendContent("function togglePath() { showPath = !showPath; }");
  
  server.sendContent("setInterval(updateMap, 200);");
  server.sendContent("updateMap();");
  server.sendContent("</script>");
  
  server.sendContent("</div></body></html>");
  server.sendContent(""); // End chunked transfer
}

void handleNotFound() {
  server.send(404, "text/plain", "Page not found");
}

void sendParametersToSTM32() {
  DynamicJsonDocument doc(1024);
  
  // Robot control state
  doc["robotRunning"] = robotRunning;
  
  // PID parameters
  doc["baseKp"] = robotParams.baseKp;
  doc["baseKi"] = robotParams.baseKi;
  doc["baseKd"] = robotParams.baseKd;
  doc["maxKp"] = robotParams.maxKp;
  doc["maxKi"] = robotParams.maxKi;
  doc["maxKd"] = robotParams.maxKd;
  doc["baseSpeed"] = robotParams.baseSpeed;
  doc["minThrottle"] = robotParams.minThrottle;
  doc["maxThrottle"] = robotParams.maxThrottle;
  doc["maxCorrection"] = robotParams.maxCorrection;
  doc["errorDeadband"] = robotParams.errorDeadband;
  doc["throttleIncrement"] = robotParams.throttleIncrement;
  doc["throttleInterval"] = robotParams.throttleInterval;
  
  JsonArray thresholds = doc.createNestedArray("sensorThresholds");
  for(int i = 0; i < 7; i++) {
    thresholds.add(robotParams.sensorThresholds[i]);
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  Serial.println(jsonString);
}

void readSTM32Status() {
  if (Serial.available()) {
    String jsonBuffer = Serial.readStringUntil('\n');
    
    // Check if this is a navigation request
    if (jsonBuffer == "NAV_REQUEST") {
      // Send navigation data immediately
      sendNavigationData();
      return;
    }
    
    // Check if this is a checkpoint request
    if (jsonBuffer == "CHECKPOINT_REQUEST") {
      // Force navigation to checkpoint mode if we have a valid checkpoint
      if (hasValidCheckpoint) {
        navState = NAV_RETURNING_TO_CHECKPOINT;
        sendNavigationData();
      }
      return;
    }
    
    // Otherwise, try to parse as JSON status
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, jsonBuffer);
    
    if (!error) {
      robotStatus.currentThrottle = doc["currentThrottle"] | robotStatus.currentThrottle;
      robotStatus.lineDetected = doc["lineDetected"] | robotStatus.lineDetected;
      robotStatus.currentPosition = doc["currentPosition"] | robotStatus.currentPosition;
      robotStatus.error = doc["error"] | robotStatus.error;
      robotStatus.inRecoveryMode = doc["inRecoveryMode"] | robotStatus.inRecoveryMode;
      robotStatus.lastUpdate = millis();
    }
  }
}

void sendNavigationData() {
  DynamicJsonDocument doc(512);
  
  // Current pose
  doc["currentX"] = currentPose.x;
  doc["currentY"] = currentPose.y;
  doc["currentYaw"] = currentPose.yaw;
  doc["timestamp"] = currentPose.timestamp;
  
  // Checkpoint information
  doc["hasCheckpoint"] = hasValidCheckpoint;
  if (hasValidCheckpoint) {
    doc["checkpointX"] = lastGoodCheckpoint.x;
    doc["checkpointY"] = lastGoodCheckpoint.y;
    doc["checkpointYaw"] = lastGoodCheckpoint.yaw;
    doc["distanceToCheckpoint"] = calculateDistanceToCheckpoint();
    doc["angleToCheckpoint"] = calculateAngleToCheckpoint();
  }
  
  // Navigation state
  doc["navigationState"] = navState;
  
  // Navigation commands for STM32
  if (navState == NAV_RETURNING_TO_CHECKPOINT && hasValidCheckpoint) {
    float distance = calculateDistanceToCheckpoint();
    float angle = calculateAngleToCheckpoint();
    
    doc["shouldReturn"] = true;
    doc["returnDistance"] = distance;
    doc["returnAngle"] = angle;
    
    // Simple navigation commands
    if (distance > 5.0) { // More than 5cm away
      if (abs(angle) > 15.0) { // Need to turn
        doc["command"] = (angle > 0) ? "TURN_LEFT" : "TURN_RIGHT";
        doc["intensity"] = min(abs(angle) / 90.0, 1.0); // Normalize turn intensity
      } else { // Go straight back
        doc["command"] = "REVERSE";
        doc["intensity"] = min(distance / 20.0, 1.0); // Normalize reverse intensity
      }
    } else { // Close to checkpoint
      doc["command"] = "ARRIVED";
      doc["intensity"] = 0.0;
      // Reset navigation state
      navState = NAV_SEARCHING;
    }
  } else {
    doc["shouldReturn"] = false;
    doc["command"] = "NONE";
    doc["intensity"] = 0.0;
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  Serial.println(jsonString);
}

void saveParameters() {
  EEPROM.put(PARAMS_ADDRESS, robotParams);
  EEPROM.commit();
}

void loadParameters() {
  RobotParameters loaded;
  EEPROM.get(PARAMS_ADDRESS, loaded);
  
  // Validate loaded parameters (basic sanity check)
  if (loaded.baseKp >= 0 && loaded.baseKp <= 10 && 
      loaded.baseSpeed >= 50 && loaded.baseSpeed <= 500) {
    robotParams = loaded;
  }
  // If validation fails, keep default values
}

String generateHTML() {
  return R"html(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Line Following Robot Controller</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            color: #333;
        }
        
        .container {
            max-width: 1200px;
            margin: 0 auto;
            padding: 20px;
        }
        
        .header {
            text-align: center;
            color: white;
            margin-bottom: 30px;
        }
        
        .header h1 {
            font-size: 2.5em;
            margin-bottom: 10px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        
        .status-bar {
            background: white;
            border-radius: 15px;
            padding: 20px;
            margin-bottom: 30px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.1);
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 20px;
        }
        
        .status-item {
            text-align: center;
        }
        
        .status-value {
            font-size: 1.8em;
            font-weight: bold;
            color: #667eea;
        }
        
        .status-label {
            color: #666;
            margin-top: 5px;
        }
        
        .connection-status {
            padding: 5px 15px;
            border-radius: 20px;
            color: white;
            font-weight: bold;
        }
        
        .connected { background: #4CAF50; }
        .disconnected { background: #f44336; }
        
        .control-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
            gap: 25px;
        }
        
        .control-panel {
            background: white;
            border-radius: 15px;
            padding: 25px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.1);
        }
        
        .panel-title {
            font-size: 1.4em;
            font-weight: bold;
            color: #667eea;
            margin-bottom: 20px;
            border-bottom: 2px solid #f0f0f0;
            padding-bottom: 10px;
        }
        
        .form-group {
            margin-bottom: 15px;
        }
        
        .form-row {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
        }
        
        label {
            display: block;
            margin-bottom: 5px;
            font-weight: 600;
            color: #555;
        }
        
        input[type="number"], input[type="range"] {
            width: 100%;
            padding: 10px;
            border: 2px solid #e0e0e0;
            border-radius: 8px;
            font-size: 1em;
            transition: border-color 0.3s;
        }
        
        input[type="number"]:focus, input[type="range"]:focus {
            outline: none;
            border-color: #667eea;
        }
        
        .slider-container {
            display: flex;
            align-items: center;
            gap: 10px;
        }
        
        .slider-value {
            min-width: 60px;
            text-align: center;
            font-weight: bold;
            color: #667eea;
        }
        
        .btn {
            padding: 12px 25px;
            border: none;
            border-radius: 8px;
            font-size: 1em;
            font-weight: bold;
            cursor: pointer;
            transition: all 0.3s;
            margin: 5px;
        }
        
        .btn-primary {
            background: #667eea;
            color: white;
        }
        
        .btn-success {
            background: #4CAF50;
            color: white;
        }
        
        .btn-warning {
            background: #ff9800;
            color: white;
        }
        
        .btn-danger {
            background: #f44336;
            color: white;
        }
        
        .btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 4px 12px rgba(0,0,0,0.2);
        }
        
        .btn-group {
            display: flex;
            flex-wrap: wrap;
            gap: 10px;
            margin-top: 20px;
        }
        
        .sensor-grid {
            display: grid;
            grid-template-columns: repeat(7, 1fr);
            gap: 10px;
            margin-top: 15px;
        }
        
        .sensor-input {
            text-align: center;
        }
        
        .sensor-label {
            font-size: 0.8em;
            margin-bottom: 5px;
            color: #666;
        }
        
        .toast {
            position: fixed;
            top: 20px;
            right: 20px;
            padding: 15px 25px;
            border-radius: 8px;
            color: white;
            font-weight: bold;
            z-index: 1000;
            opacity: 0;
            transform: translateX(100%);
            transition: all 0.3s;
        }
        
        .toast.show {
            opacity: 1;
            transform: translateX(0);
        }
        
        .toast-success { background: #4CAF50; }
        .toast-error { background: #f44336; }
        
        @media (max-width: 768px) {
            .control-grid {
                grid-template-columns: 1fr;
            }
            
            .form-row {
                grid-template-columns: 1fr;
            }
            
            .sensor-grid {
                grid-template-columns: repeat(4, 1fr);
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🤖 Line Following Robot Controller</h1>
            <p>Real-time parameter adjustment and monitoring</p>
            <p style="font-size: 0.9em; margin-top: 10px;">📡 Access Point: Robot_Controller | 🌐 IP: 192.168.4.1</p>
        </div>
        
        <div class="status-bar">
            <div class="status-item">
                <div class="status-value" id="throttle">220</div>
                <div class="status-label">Current Throttle</div>
            </div>
            <div class="status-item">
                <div class="status-value" id="position">3000</div>
                <div class="status-label">Line Position</div>
            </div>
            <div class="status-item">
                <div class="status-value" id="error">0.0</div>
                <div class="status-label">PID Error</div>
            </div>
            <div class="status-item">
                <div class="status-value">
                    <span id="connection-status" class="connection-status disconnected">Disconnected</span>
                </div>
                <div class="status-label">Robot Status</div>
            </div>
            <div class="status-item">
                <div class="status-value" id="clients">0</div>
                <div class="status-label">Connected Clients</div>
            </div>
            <div class="status-item">
                <div class="status-value">
                    <button id="startStopBtn" class="btn btn-success" onclick="toggleRobotState()">START</button>
                </div>
                <div class="status-label">Robot Control</div>
            </div>
        </div>
        
        <div class="control-grid">
            <!-- PID Control Panel -->
            <div class="control-panel">
                <div class="panel-title">🎛️ PID Parameters</div>
                
                <h4 style="color: #667eea; margin-bottom: 15px;">Base Speed PID (Low Speed)</h4>
                <div class="form-row">
                    <div class="form-group">
                        <label>Kp (Proportional)</label>
                        <input type="number" id="baseKp" min="0" max="1" step="0.01" value="0.1" style="width:100px;">
                    </div>
                    <div class="form-group">
                        <label>Ki (Integral)</label>
                        <input type="number" id="baseKi" min="0" max="0.1" step="0.001" value="0.02" style="width:100px;">
                    </div>
                </div>
                <div class="form-group">
                    <label>Kd (Derivative)</label>
                    <input type="number" id="baseKd" min="0" max="0.2" step="0.005" value="0.05" style="width:100px;">
                </div>
                
                <h4 style="color: #667eea; margin: 20px 0 15px;">Max Speed PID (High Speed)</h4>
                <div class="form-row">
                    <div class="form-group">
                        <label>Kp (Proportional)</label>
                        <input type="number" id="maxKp" min="0" max="1" step="0.01" value="0.1" style="width:100px;">
                    </div>
                    <div class="form-group">
                        <label>Ki (Integral)</label>
                        <input type="number" id="maxKi" min="0" max="0.1" step="0.001" value="0.02" style="width:100px;">
                    </div>
                </div>
                <div class="form-group">
                    <label>Kd (Derivative)</label>
                    <input type="number" id="maxKd" min="0" max="0.2" step="0.005" value="0.05" style="width:100px;">
                </div>
                
                <button class="btn btn-primary" onclick="updatePID()">Update PID</button>
            </div>
            
            <!-- Motor Control Panel -->
            <div class="control-panel">
                <div class="panel-title">⚙️ Motor & Control</div>
                
                <div class="form-row">
                    <div class="form-group">
                        <label>Base Speed</label>
                        <input type="number" id="baseSpeed" min="100" max="300" value="220" style="width:100px;">
                    </div>
                    <div class="form-group">
                        <label>Min Throttle</label>
                        <input type="number" id="minThrottle" min="100" max="300" value="220" style="width:100px;">
                    </div>
                </div>
                
                <div class="form-row">
                    <div class="form-group">
                        <label>Max Throttle</label>
                        <input type="number" id="maxThrottle" min="200" max="400" value="250" style="width:100px;">
                    </div>
                    <div class="form-group">
                        <label>Max Correction</label>
                        <input type="number" id="maxCorrection" min="100" max="800" value="400" style="width:100px;">
                    </div>
                </div>
                
                <div class="form-row">
                    <div class="form-group">
                        <label>Error Deadband</label>
                        <input type="number" id="errorDeadband" min="0" max="50" value="10" style="width:100px;">
                    </div>
                    <div class="form-group">
                        <label>Throttle Increment</label>
                        <input type="number" id="throttleIncrement" min="1" max="50" value="10" style="width:100px;">
                    </div>
                </div>
                
                <div class="form-group">
                    <label>Throttle Interval (ms)</label>
                    <input type="number" id="throttleInterval" min="50" max="1000" step="50" value="200" style="width:100px;">
                </div>
                
                <button class="btn btn-primary" onclick="updateMotor()">Update Motor</button>
            </div>
            
            <!-- Sensor Thresholds Panel -->
            <div class="control-panel">
                <div class="panel-title">📊 Sensor Thresholds</div>
                <p style="color: #666; margin-bottom: 15px;">Adjust detection thresholds for each sensor</p>
                
                <div class="sensor-grid">
                    <div class="sensor-input">
                        <div class="sensor-label">L3</div>
                        <input type="number" id="sensor0" min="0" max="4095" value="2550">
                    </div>
                    <div class="sensor-input">
                        <div class="sensor-label">L2</div>
                        <input type="number" id="sensor1" min="0" max="4095" value="2880">
                    </div>
                    <div class="sensor-input">
                        <div class="sensor-label">L1</div>
                        <input type="number" id="sensor2" min="0" max="4095" value="3000">
                    </div>
                    <div class="sensor-input">
                        <div class="sensor-label">M0</div>
                        <input type="number" id="sensor3" min="0" max="4095" value="3500">
                    </div>
                    <div class="sensor-input">
                        <div class="sensor-label">R1</div>
                        <input type="number" id="sensor4" min="0" max="4095" value="3150">
                    </div>
                    <div class="sensor-input">
                        <div class="sensor-label">R2</div>
                        <input type="number" id="sensor5" min="0" max="4095" value="3125">
                    </div>
                    <div class="sensor-input">
                        <div class="sensor-label">R3</div>
                        <input type="number" id="sensor6" min="0" max="4095" value="2600">
                    </div>
                </div>
                
                <button class="btn btn-primary" onclick="updateSensors()" style="margin-top: 20px;">Update Sensors</button>
            </div>
            
            <!-- System Control Panel -->
            <div class="control-panel">
                <div class="panel-title">🔧 System Control</div>
                
                <div class="btn-group">
                    <button class="btn btn-success" onclick="saveParameters()">💾 Save to EEPROM</button>
                    <button class="btn btn-warning" onclick="loadParameters()">📁 Load from EEPROM</button>
                    <button class="btn btn-danger" onclick="resetParameters()">🔄 Reset to Defaults</button>
                </div>
                
                <div style="margin-top: 20px; padding: 15px; background: #f8f9fa; border-radius: 8px;">
                    <h4 style="color: #667eea; margin-bottom: 10px;">📈 Quick Tuning Tips</h4>
                    <ul style="color: #666; line-height: 1.6;">
                        <li><strong>Kp:</strong> Increase for faster response, decrease if oscillating</li>
                        <li><strong>Ki:</strong> Helps eliminate steady-state error</li>
                        <li><strong>Kd:</strong> Reduces overshoot and oscillation</li>
                        <li><strong>Base Speed:</strong> Starting speed for line following</li>
                        <li><strong>Max Correction:</strong> Maximum steering adjustment</li>
                    </ul>
                </div>
            </div>
        </div>
    </div>
    
    <div class="toast" id="toast"></div>
    
    <script>
        // Initialize inputs and update displays
        document.addEventListener('DOMContentLoaded', function() {
            setupInputs();
            loadCurrentParameters();
            startStatusUpdates();
        });
        
        function setupInputs() {
            // Input fields don't need special setup like sliders did
            // Values are directly accessible via .value property
        }
        
        function loadCurrentParameters() {
            fetch('/api/parameters')
                .then(response => response.json())
                .then(data => {
                    // Update input fields
                    Object.keys(data).forEach(key => {
                        const element = document.getElementById(key);
                        if (element && key !== 'sensorThresholds') {
                            element.value = data[key];
                        }
                    });
                    
                    // Update sensor thresholds
                    if (data.sensorThresholds) {
                        for (let i = 0; i < 7; i++) {
                            const sensorInput = document.getElementById('sensor' + i);
                            if (sensorInput) {
                                sensorInput.value = data.sensorThresholds[i];
                            }
                        }
                    }
                })
                .catch(error => console.error('Error loading parameters:', error));
        }
        
        function startStatusUpdates() {
            setInterval(updateStatus, 1000);
            updateStatus(); // Initial call
        }
        
        function updateStatus() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('throttle').textContent = data.currentThrottle;
                    document.getElementById('position').textContent = data.currentPosition.toFixed(1);
                    document.getElementById('error').textContent = data.error.toFixed(2);
                    document.getElementById('clients').textContent = data.connectedClients || 0;
                    
                    // Update START/STOP button
                    const startStopBtn = document.getElementById('startStopBtn');
                    if (data.robotRunning) {
                        startStopBtn.textContent = 'STOP';
                        startStopBtn.className = 'btn btn-danger';
                    } else {
                        startStopBtn.textContent = 'START';
                        startStopBtn.className = 'btn btn-success';
                    }
                    
                    const connectionStatus = document.getElementById('connection-status');
                    if (data.connected) {
                        if (!data.robotRunning) {
                            connectionStatus.textContent = 'Stopped';
                            connectionStatus.className = 'connection-status disconnected';
                        } else {
                            connectionStatus.textContent = data.lineDetected ? 'Line Detected' : 
                                                        data.inRecoveryMode ? 'Recovery Mode' : 'Running';
                            connectionStatus.className = 'connection-status connected';
                        }
                    } else {
                        connectionStatus.textContent = 'Disconnected';
                        connectionStatus.className = 'connection-status disconnected';
                    }
                })
                .catch(error => {
                    console.error('Error updating status:', error);
                    const connectionStatus = document.getElementById('connection-status');
                    connectionStatus.textContent = 'Connection Error';
                    connectionStatus.className = 'connection-status disconnected';
                });
        }
        
        function updatePID() {
            const pidData = {
                baseKp: parseFloat(document.getElementById('baseKp').value),
                baseKi: parseFloat(document.getElementById('baseKi').value),
                baseKd: parseFloat(document.getElementById('baseKd').value),
                maxKp: parseFloat(document.getElementById('maxKp').value),
                maxKi: parseFloat(document.getElementById('maxKi').value),
                maxKd: parseFloat(document.getElementById('maxKd').value)
            };
            
            fetch('/api/pid', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(pidData)
            })
            .then(response => response.json())
            .then(data => showToast(data.status, 'success'))
            .catch(error => showToast('Error updating PID', 'error'));
        }
        
        function updateMotor() {
            const motorData = {
                baseSpeed: parseInt(document.getElementById('baseSpeed').value),
                minThrottle: parseInt(document.getElementById('minThrottle').value),
                maxThrottle: parseInt(document.getElementById('maxThrottle').value),
                maxCorrection: parseInt(document.getElementById('maxCorrection').value),
                errorDeadband: parseInt(document.getElementById('errorDeadband').value),
                throttleIncrement: parseInt(document.getElementById('throttleIncrement').value),
                throttleInterval: parseInt(document.getElementById('throttleInterval').value)
            };
            
            fetch('/api/motor', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(motorData)
            })
            .then(response => response.json())
            .then(data => showToast(data.status, 'success'))
            .catch(error => showToast('Error updating motor parameters', 'error'));
        }
        
        function updateSensors() {
            const sensorThresholds = [];
            for (let i = 0; i < 7; i++) {
                sensorThresholds.push(parseInt(document.getElementById('sensor' + i).value));
            }
            
            fetch('/api/sensors', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ sensorThresholds })
            })
            .then(response => response.json())
            .then(data => showToast(data.status, 'success'))
            .catch(error => showToast('Error updating sensor thresholds', 'error'));
        }
        
        function saveParameters() {
            fetch('/api/save', { method: 'POST' })
                .then(response => response.json())
                .then(data => showToast(data.status, 'success'))
                .catch(error => showToast('Error saving parameters', 'error'));
        }
        
        function loadParameters() {
            fetch('/api/load', { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    showToast(data.status, 'success');
                    setTimeout(loadCurrentParameters, 500); // Reload UI after 500ms
                })
                .catch(error => showToast('Error loading parameters', 'error'));
        }
        
        function resetParameters() {
            if (confirm('Are you sure you want to reset all parameters to defaults?')) {
                fetch('/api/reset', { method: 'POST' })
                    .then(response => response.json())
                    .then(data => {
                        showToast(data.status, 'success');
                        setTimeout(loadCurrentParameters, 500); // Reload UI after 500ms
                    })
                    .catch(error => showToast('Error resetting parameters', 'error'));
            }
        }
        
        function toggleRobotState() {
            const startStopBtn = document.getElementById('startStopBtn');
            const isRunning = startStopBtn.textContent === 'STOP';
            
            const endpoint = isRunning ? '/api/stop' : '/api/start';
            
            fetch(endpoint, { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    showToast(data.status, 'success');
                    updateStatus(); // Immediately update status
                })
                .catch(error => {
                    showToast('Error controlling robot', 'error');
                });
        }
        
        function showToast(message, type) {
            const toast = document.getElementById('toast');
            toast.textContent = message;
            toast.className = `toast toast-${type} show`;
            
            setTimeout(() => {
                toast.className = 'toast';
            }, 3000);
        }
    </script>
</body>
</html>
)html";
}
