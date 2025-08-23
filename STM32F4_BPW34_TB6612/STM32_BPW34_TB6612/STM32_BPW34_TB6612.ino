// Arduino Line Following Robot
// PID control with smart turn detection for 90-degree turns
// Enhanced edge sensor memory for sharp turn recovery
// Dynamic throttle system: starts at 140, increases gradually when on line, resets when line lost
// Adaptive PID: automatically adjusts PID gains based on current throttle for optimal control
// Non-blocking serial communication with ESP8266
// CURVED SENSOR ARRAY OPTIMIZATION - Enhanced for arc-shaped sensor layout

#include <Arduino.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>

// HardwareSerial instance for ESP8266 communication
HardwareSerial Serial1(USART1);

// ===== CURVED SENSOR ARRAY GEOMETRY =====
// 7-sensor array: L3, L2, L1, M0(center), R1, R2, R3
const uint8_t sensorPins[7] = { PA4, PA5, PA6, PA0, PA1, PA2, PA3 };

// Curved sensor array constants
#define CURVED_WEIGHT_MULTIPLIER 1.3
#define CURVED_ARRAY_CENTER 3000.0  // Center position for 7-sensor array

// Curved sensor array parameters (inward curve - sensors curve toward robot center)
// Distances and angles from center (M0) to each sensor
struct SensorGeometry {
  float distance;  // Distance from center in mm
  float angle;     // Angle from center in degrees (positive = left, negative = right)
  float weight;    // Position weight for line following
  float position;  // Calculated position value for this sensor
};

// Curved sensor geometry - corrected calculations
// Position values are calculated based on sensor spacing and curve geometry
const SensorGeometry sensorGeometry[7] = {
  // L3 (leftmost) - position 0
  {46.5, 99.27, 6000, 0.0},   
  // L2 - position 1000
  {23.019, 58.61, 5000, 1000.0}, 
  // L1 - position 2000
  {23.019, 40.66, 4000, 2000.0}, 
  // M0 (center) - position 3000
  {0.0, 0.0, 3000, 3000.0},      
  // R1 - position 4000
  {21.76, -36.69, 2000, 4000.0}, 
  // R2 - position 5000
  {40.966, -72.07, 1000, 5000.0}, 
  // R3 (rightmost) - position 6000
  {64.0, -111.4, 0, 6000.0}      
};

// Dynamic threshold values - can be updated via web interface
int sensorThresholds[7] = { 2550, 2880, 3000, 3500, 3150, 3125, 2600 };

// Motor driver pins
#define PWMA PB13
#define AIN2 PB14
#define AIN1 PB15
#define BIN1 PA8
#define BIN2 PB3
#define PWMB PB4

// LED pin for calibration indication
#define LED_PIN PC13

// ===== TIMING CONSTANTS =====
#define SAMPLE_INTERVAL 1  // 1ms ultra-fast sensor reading
#define PID_INTERVAL 1     // 1ms ultra-fast PID for micro-corrections

// ===== PID VARIABLES =====
// Curved array optimized PID values - enhanced for better turn detection and precision
float baseKp = 0.12;  // Slightly higher response for curved array's better turn detection
float baseKi = 0.025; // Slightly higher to eliminate offset with curved geometry
float baseKd = 0.06;  // Slightly higher damping for smoother curved path following

// PID values for maximum throttle (250) - optimized for curved array high-speed performance
float maxKp = baseKp;  // Higher proportional gain for curved array's enhanced precision
float maxKi = baseKi;  // Higher integral gain for curved array stability
float maxKd = baseKd;  // Higher derivative gain for curved array's smoother control

// Current adaptive PID values (will be calculated based on throttle)
float Kp, Ki, Kd;
int baseSpeed = 140;  // Starting base speed - will be managed by throttle

// ===== THROTTLE VARIABLES =====
int currentThrottle = 140;                    // Current throttle level (starts at base)
int MIN_THROTTLE = 140;                       // Minimum throttle (base speed) - now updatable
int MAX_THROTTLE = 255;                       // Maximum throttle limit - now updatable
int THROTTLE_INCREMENT = 10;                  // Speed increase per step - now updatable
unsigned long THROTTLE_INTERVAL = 200;        // 200ms between throttle increases - now updatable
unsigned long lastThrottleTime = 0;
unsigned long onLineStartTime = 0;
bool wasOnLine = false;

// ===== CONTROL PARAMETERS =====
int MAX_CORRECTION = 400;  // Reduced steering to prevent over-correction - now updatable
int ERROR_DEADBAND = 10;   // Larger deadband to reduce wobbling on straight paths - now updatable

float error = 0, lastError = 0, integral = 0;
unsigned long lastPidTime = 0;
unsigned long lastSampleTime = 0;

// ===== LINE DETECTION VARIABLES =====
bool lineDetected = false;
unsigned long lineDetectedTime = 0;
float currentPosition = 3000;  // Center position for 7-sensor array (sensor 3, PA0)
int sampleCount = 0;

// ===== DOTTED LINE DETECTION VARIABLES =====
bool inDottedLineMode = false;                    // Flag to indicate if robot is in dotted line traversal mode
unsigned long dottedLineStartTime = 0;            // Timestamp when dotted line mode was entered
unsigned long lastLineDetectionTime = 0;          // Timestamp of the last successful line detection
int dottedLineForwardCount = 0;                   // Counter for how many forward steps taken in dotted line mode
const int MAX_DOTTED_FORWARD_STEPS = 15;          // Maximum steps to move forward before giving up (prevents going too far off course)
const unsigned long DOTTED_LINE_TIMEOUT = 800;    // 800ms timeout - if no line found within this time, exit dotted line mode

// ===== SMART TURN VARIABLES =====
bool leftEdgeDetected = false;    // Sensor 0 detected black
bool rightEdgeDetected = false;   // Sensor 7 detected black
unsigned long leftEdgeTime = 0;   // When left edge was last detected
unsigned long rightEdgeTime = 0;  // When right edge was last detected
int lastTurnDirection = 0;        // -1 = left, 1 = right, 0 = straight
bool inRecoveryMode = false;      // Currently trying to recover line
unsigned long recoveryStartTime = 0;

// ===== SERIAL COMMUNICATION VARIABLES =====
String serialBuffer = "";
unsigned long lastSerialWrite = 0;
const unsigned long SERIAL_WRITE_INTERVAL = 500;  // Send status every 500ms

// ===== NAVIGATION VARIABLES =====
struct NavigationData {
  bool shouldReturn = false;
  float returnDistance = 0.0;
  float returnAngle = 0.0;
  String command = "NONE";
  float intensity = 0.0;
  bool hasCheckpoint = false;
  unsigned long lastUpdate = 0;
} navData;

unsigned long lastNavigationRequest = 0;
const unsigned long NAVIGATION_REQUEST_INTERVAL = 200;  // Request navigation data every 200ms

// ===== PARAMETER UPDATE VARIABLES =====
volatile bool parametersUpdated = false;
volatile bool robotRunning = true;  // Default state is START

// Structure to hold all updatable parameters
struct RobotParameters {
  float baseKp;
  float baseKi; 
  float baseKd;
  float maxKp;
  float maxKi;
  float maxKd;
  int baseSpeed;
  int minThrottle;
  int maxThrottle;
  int throttleIncrement;
  unsigned long throttleInterval;
  int maxCorrection;
  int errorDeadband;
  int sensorThresholds[7];
} robotParams;

// ===== PID ADAPTATION FUNCTIONS =====
void updateAdaptivePID() {
  // Calculate throttle ratio (0.0 = MIN_THROTTLE, 1.0 = MAX_THROTTLE)
  float throttleRatio = (float)(currentThrottle - MIN_THROTTLE) / (MAX_THROTTLE - MIN_THROTTLE);

  // Linear interpolation between base and max PID values
  // At low throttle: use baseKp/Ki/Kd for aggressive corrections
  // At high throttle: use maxKp/Ki/Kd for smooth, stable control
  Kp = baseKp + (maxKp - baseKp) * throttleRatio;
  Ki = baseKi + (maxKi - baseKi) * throttleRatio;
  Kd = baseKd + (maxKd - baseKd) * throttleRatio;
}

int getAdaptiveDeadband() {
  // Increase deadband with throttle for smoother high-speed performance
  float throttleRatio = (float)(currentThrottle - MIN_THROTTLE) / (MAX_THROTTLE - MIN_THROTTLE);
  return ERROR_DEADBAND - (int)(throttleRatio * 35);  // 25-40 deadband range
}

// ===== PARAMETER MANAGEMENT FUNCTIONS =====
void initializeParameters() {
  robotParams.baseKp = baseKp;
  robotParams.baseKi = baseKi;
  robotParams.baseKd = baseKd;
  robotParams.maxKp = maxKp;
  robotParams.maxKi = maxKi;
  robotParams.maxKd = maxKd;
  robotParams.baseSpeed = baseSpeed;
  robotParams.minThrottle = MIN_THROTTLE;
  robotParams.maxThrottle = MAX_THROTTLE;
  robotParams.throttleIncrement = THROTTLE_INCREMENT;
  robotParams.throttleInterval = THROTTLE_INTERVAL;
  robotParams.maxCorrection = MAX_CORRECTION;
  robotParams.errorDeadband = ERROR_DEADBAND;
  
  // Copy sensor thresholds
  for(int i = 0; i < 7; i++) {
    robotParams.sensorThresholds[i] = sensorThresholds[i];
  }
}

void updateGlobalParameters() {
  // Update PID parameters
  baseKp = robotParams.baseKp;
  baseKi = robotParams.baseKi;
  baseKd = robotParams.baseKd;
  maxKp = robotParams.maxKp;
  maxKi = robotParams.maxKi;
  maxKd = robotParams.maxKd;
  
  // Update motor and control parameters
  baseSpeed = robotParams.baseSpeed;
  MIN_THROTTLE = robotParams.minThrottle;
  MAX_THROTTLE = robotParams.maxThrottle;
  THROTTLE_INCREMENT = robotParams.throttleIncrement;
  THROTTLE_INTERVAL = robotParams.throttleInterval;
  MAX_CORRECTION = robotParams.maxCorrection;
  ERROR_DEADBAND = robotParams.errorDeadband;
  
  // Update sensor thresholds
  for(int i = 0; i < 7; i++) {
    sensorThresholds[i] = robotParams.sensorThresholds[i];
  }
  
  // Reset current throttle to new minimum if it's below the new minimum
  if(currentThrottle < MIN_THROTTLE) {
    currentThrottle = MIN_THROTTLE;
  }
  // Cap current throttle to new maximum if it's above the new maximum
  if(currentThrottle > MAX_THROTTLE) {
    currentThrottle = MAX_THROTTLE;
  }
  
  updateAdaptivePID(); // Recalculate PID values
  parametersUpdated = false;
}



// ===== CURVED SENSOR ARRAY READING FUNCTIONS =====
float readLinePosition() {
  float weightedSum = 0;
  float totalWeight = 0;
  int blackCount = 0;
  bool sensorsDetected[7] = {false, false, false, false, false, false, false};

  // Read all 7 sensors once
  for (int i = 0; i < 7; i++) {
    int value = analogRead(sensorPins[i]);

    // Use fixed threshold for each sensor
    // Lower values indicate black line (we follow black)
    if (value < sensorThresholds[i]) {  // black line detected
      sensorsDetected[i] = true;
      blackCount++;

      // Apply curved array weighting based on sensor geometry
      float sensorWeight = sensorGeometry[i].weight;
      
      // Boost weight for outer sensors due to curve geometry
      if (i == 0 || i == 6) {  // L3 or R3 (outermost sensors)
        sensorWeight *= CURVED_WEIGHT_MULTIPLIER * 1.5;  // Extra boost for edge detection
      } else if (i == 1 || i == 5) {  // L2 or R2 (second outermost)
        sensorWeight *= CURVED_WEIGHT_MULTIPLIER * 1.2;
      } else if (i == 2 || i == 4) {  // L1 or R1 (inner sensors)
        sensorWeight *= CURVED_WEIGHT_MULTIPLIER;
      }
      // Center sensor (M0) keeps base weight

      // Use the sensor's position value for weighted calculation
      weightedSum += sensorGeometry[i].position * sensorWeight;
      totalWeight += sensorWeight;

      // Enhanced edge sensor detection for curved array
      if (i == 0 || i == 1) {  // Leftmost sensors (L3, L2) - more sensitive due to curve
        leftEdgeDetected = true;
        leftEdgeTime = millis();
      }
      if (i == 5 || i == 6) {  // Rightmost sensors (R2, R3) - more sensitive due to curve
        rightEdgeDetected = true;
        rightEdgeTime = millis();
      }
    }
  }

  // Return position if we detected black line, otherwise return -1 to indicate no line
  if (blackCount > 0) {
    // Calculate weighted average position for curved array
    float averagePosition = weightedSum / totalWeight;
    
    // Apply curved array correction factor based on detected sensors
    float correctionFactor = 1.0;
    
    // If outer sensors are detected, apply curve correction
    if (sensorsDetected[0] || sensorsDetected[6]) {  // L3 or R3 detected
      correctionFactor = 1.15;  // Boost position calculation for outer curve
    } else if (sensorsDetected[1] || sensorsDetected[5]) {  // L2 or R2 detected
      correctionFactor = 1.08;  // Moderate boost for inner curve
    }
    
    return averagePosition * correctionFactor;
  } else {
    return -1;  // No line detected
  }
}

void stopMotors() {
  leftMotor(0);
  rightMotor(0);
}

void sharpLeftTurn() {
  leftMotor(-200);  // Maximum reverse left motor
  rightMotor(200);  // Maximum forward right motor
}

void sharpRightTurn() {
  leftMotor(200);    // Maximum forward left motor
  rightMotor(-200);  // Maximum reverse right motor
}

void fastLeftSearch() {
  leftMotor(50);    // Slow forward left
  rightMotor(200);  // Fast forward right
}

void fastRightSearch() {
  leftMotor(200);  // Fast forward left
  rightMotor(50);  // Slow forward right
}

// ===== NON-BLOCKING SERIAL FUNCTIONS =====
void handleSerialCommunication() {
  // Non-blocking serial read
  while(Serial1.available() && serialBuffer.length() < 1500) {
    char c = Serial1.read();
    
    if(c == '\n' || c == '\r') {
      if(serialBuffer.length() > 0) {
        // Parse JSON and update parameters
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, serialBuffer);
        
        if(!error) {
          // Check if this is navigation data
          if(doc.containsKey("shouldReturn")) {
            // This is navigation data from ESP8266
            navData.shouldReturn = doc["shouldReturn"] | false;
            navData.returnDistance = doc["returnDistance"] | 0.0;
            navData.returnAngle = doc["returnAngle"] | 0.0;
            navData.command = doc["command"] | "NONE";
            navData.intensity = doc["intensity"] | 0.0;
            navData.hasCheckpoint = doc["hasCheckpoint"] | false;
            navData.lastUpdate = millis();
          }
          // Check if this is parameter data
          else if(doc.containsKey("robotRunning")) {
            // Update robot running state
            robotRunning = doc["robotRunning"];
            
            // Update parameters from JSON
            if(doc.containsKey("baseKp")) robotParams.baseKp = doc["baseKp"];
            if(doc.containsKey("baseKi")) robotParams.baseKi = doc["baseKi"];
            if(doc.containsKey("baseKd")) robotParams.baseKd = doc["baseKd"];
            if(doc.containsKey("maxKp")) robotParams.maxKp = baseKp;
            if(doc.containsKey("maxKi")) robotParams.maxKi = baseKi;
            if(doc.containsKey("maxKd")) robotParams.maxKd = baseKd;
            if(doc.containsKey("baseSpeed")) robotParams.baseSpeed = doc["baseSpeed"];
            if(doc.containsKey("minThrottle")) robotParams.minThrottle = doc["minThrottle"];
            if(doc.containsKey("maxThrottle")) robotParams.maxThrottle = doc["maxThrottle"];
            if(doc.containsKey("maxCorrection")) robotParams.maxCorrection = doc["maxCorrection"];
            if(doc.containsKey("errorDeadband")) robotParams.errorDeadband = doc["errorDeadband"];
            if(doc.containsKey("throttleIncrement")) robotParams.throttleIncrement = doc["throttleIncrement"];
            if(doc.containsKey("throttleInterval")) robotParams.throttleInterval = doc["throttleInterval"];
            
            // Update sensor thresholds if provided
            if(doc.containsKey("sensorThresholds")) {
              JsonArray thresholds = doc["sensorThresholds"];
              for(int i = 0; i < 7 && i < thresholds.size(); i++) {
                robotParams.sensorThresholds[i] = thresholds[i];
              }
            }
            
            parametersUpdated = true;
          }
        }
        serialBuffer = "";
      }
      break; // Exit while loop after processing one complete message
    } else if(c >= 32 && c <= 126) { // Only accept printable characters
      serialBuffer += c;
    }
  }
  
  // Clear buffer if it gets too large (corrupted data)
  if(serialBuffer.length() >= 1500) {
    serialBuffer = "";
  }
}

void sendStatusToESP8266() {
  unsigned long currentTime = millis();
  
  // Send status every SERIAL_WRITE_INTERVAL ms
  if(currentTime - lastSerialWrite >= SERIAL_WRITE_INTERVAL) {
    DynamicJsonDocument doc(512);
    
    // Add runtime status
    doc["currentThrottle"] = currentThrottle;
    doc["lineDetected"] = lineDetected;
    doc["currentPosition"] = currentPosition;
    doc["error"] = error;
    doc["inRecoveryMode"] = inRecoveryMode;
    doc["inDottedLineMode"] = inDottedLineMode;           // Current dotted line mode status
    doc["dottedLineForwardCount"] = dottedLineForwardCount; // Number of forward steps taken in dotted line mode
    doc["robotRunning"] = robotRunning;
    
    // Add enhanced navigation feedback for better ESP8266 dead reckoning
    doc["leftEdgeDetected"] = leftEdgeDetected;
    doc["rightEdgeDetected"] = rightEdgeDetected;
    doc["lastTurnDirection"] = lastTurnDirection;
    doc["navigationActive"] = navData.shouldReturn;
    doc["currentSpeed"] = currentThrottle;  // Help ESP8266 estimate velocity
    
    String jsonString;
    serializeJson(doc, jsonString);
    
    // Send status to ESP8266
    Serial1.println(jsonString);
    
    lastSerialWrite = currentTime;
  }
}

void requestNavigationData() {
  unsigned long currentTime = millis();
  
  // Request navigation data from ESP8266 at regular intervals
  if(currentTime - lastNavigationRequest >= NAVIGATION_REQUEST_INTERVAL) {
    // Send navigation request command
    Serial1.println("NAV_REQUEST");
    lastNavigationRequest = currentTime;
  }
}

void requestCheckpointReturn() {
  // Force ESP8266 to provide checkpoint navigation commands
  Serial1.println("CHECKPOINT_REQUEST");
}

void executeNavigationCommand() {
  // Only execute navigation commands if we have recent data and should return
  unsigned long currentTime = millis();
  if(!navData.shouldReturn || (currentTime - navData.lastUpdate) > 1000) {
    return; // No navigation command or data too old
  }
  
  // Execute the navigation command with improved precision
  if(navData.command == "TURN_LEFT") {
    // Turn left with intensity-based speed
    int turnSpeed = (int)(150 * navData.intensity + 100); // 100-250 range
    leftMotor(-turnSpeed);
    rightMotor(turnSpeed);
  } 
  else if(navData.command == "TURN_RIGHT") {
    // Turn right with intensity-based speed
    int turnSpeed = (int)(150 * navData.intensity + 100); // 100-250 range
    leftMotor(turnSpeed);
    rightMotor(-turnSpeed);
  }
  else if(navData.command == "REVERSE") {
    // Move backward with intensity-based speed
    int reverseSpeed = (int)(100 * navData.intensity + 50); // 50-150 range
    leftMotor(-reverseSpeed);
    rightMotor(-reverseSpeed);
  }
  else if(navData.command == "ARRIVED") {
    // Arrived at checkpoint - stop and switch back to line search
    stopMotors();
    inRecoveryMode = false;
    navData.shouldReturn = false;
    
    // Reset edge detection memory for fresh line search
    leftEdgeDetected = false;
    rightEdgeDetected = false;
    lastTurnDirection = 0;
  }
  else if(navData.command == "NONE") {
    // No specific navigation command - continue with normal recovery
    return;
  }
}

// ===== MOTOR FUNCTIONS =====
void leftMotor(int speed) {
  if (speed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speed = -speed;
  }
  analogWrite(PWMA, constrain(speed, 0, 255));
}

void rightMotor(int speed) {
  if (speed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    speed = -speed;
  }
  analogWrite(PWMB, constrain(speed, 0, 255));
}

// ===== MAIN LINE FOLLOWING LOGIC =====
void executeLineFollowing() {
  // Check for parameter updates
  if(parametersUpdated) {
    updateGlobalParameters();
  }
  
  // If robot is stopped, just stop motors and return
  if(!robotRunning) {
    stopMotors();
    return;
  }
  
  unsigned long currentTime = millis();

  // Read sensors at regular intervals
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentTime;

    // Get line position with multiple readings
    float rawPosition = readLinePosition();

    if (rawPosition >= 0) {
      // Line detected - use direct position
      currentPosition = rawPosition;
      lineDetected = true;
      lineDetectedTime = currentTime;
      lastLineDetectionTime = currentTime;  // Track when we last detected line (for dotted line detection)
    } else {
      // No line detected
      lineDetected = false;
    }
  }

  // Execute PID control at regular intervals
  if (currentTime - lastPidTime >= PID_INTERVAL) {
    lastPidTime = currentTime;

    // Check if line was detected recently (within last 100ms)
    if (lineDetected && (currentTime - lineDetectedTime) < 100) {
      // Line is detected - normal PID control
      inRecoveryMode = false;
      
      // Reset dotted line mode if we were in it
      // This ensures clean state when we successfully find the line again
      if (inDottedLineMode) {
        inDottedLineMode = false;        // Exit dotted line mode
        dottedLineForwardCount = 0;      // Reset the step counter for next time
      }

      // ===== THROTTLE MANAGEMENT =====
      // Track when robot gets on line
      if (!wasOnLine) {
        onLineStartTime = currentTime;
        wasOnLine = true;
      }

      // Gradually increase throttle if robot has been on line long enough
      if (wasOnLine && (currentTime - onLineStartTime) >= THROTTLE_INTERVAL && (currentTime - lastThrottleTime) >= THROTTLE_INTERVAL) {
        if (currentThrottle < MAX_THROTTLE) {
          currentThrottle += THROTTLE_INCREMENT;
          lastThrottleTime = currentTime;
          updateAdaptivePID();  // Update PID values for new throttle
        }
      }

      // Calculate error from center position for curved array (3000 = exactly at sensor M0, PA0)
      // Curved array provides better precision for turns, so we can use more aggressive error calculation
      error = CURVED_ARRAY_CENTER - currentPosition;

      // Apply adaptive deadband to reduce jitter from small errors
      int currentDeadband = getAdaptiveDeadband();
      if (abs(error) < currentDeadband) {
        error = 0;     // Ignore very small errors
        integral = 0;  // Reset integral when centered
      }

      // Update turn direction memory based on current position for curved array
      if (currentPosition < 2000) lastTurnDirection = -1;      // Left turn (line on left side)
      else if (currentPosition > 4000) lastTurnDirection = 1;  // Right turn (line on right side)

      // PID calculations with time-based integral
      float deltaTime = PID_INTERVAL / 1000.0;  // Convert to seconds
      integral += error * deltaTime;

      // Limit integral windup
      integral = constrain(integral, -500, 500);

      float derivative = (error - lastError) / deltaTime;
      float rawCorrection = Kp * error + Ki * integral + Kd * derivative;

      // Limit correction to prevent excessive steering that slows the robot
      float correction = constrain(rawCorrection, -MAX_CORRECTION, MAX_CORRECTION);

      lastError = error;

      // Normal PID control with forward motion using dynamic throttle
      int leftSpeed = currentThrottle - correction;
      int rightSpeed = currentThrottle + correction;

      // Allow full range of corrections for proper line following
      // Only prevent complete reversal for safety
      if (leftSpeed < -150) leftSpeed = -150;    // Allow reverse but limit
      if (rightSpeed < -150) rightSpeed = -150;  // Allow reverse but limit

      // Apply motor speeds with constraints
      leftMotor(constrain(leftSpeed, -255, 255));
      rightMotor(constrain(rightSpeed, -255, 255));

    } else {
      // No line detected - check for dotted line pattern first
      // This is the key improvement: instead of immediately turning when line is lost,
      // we first check if this might be a dotted line (brief white space)
      handleDottedLineMode();
      
      // If we're in dotted line mode, skip normal recovery
      // The dotted line handler is managing the robot's movement
      if (inDottedLineMode) {
        return;  // Continue with dotted line mode - don't enter normal recovery
      }
      
      // No line detected and not in dotted line mode - enter normal recovery mode
      // This handles cases where the line is truly lost (not just a dotted line gap)
      if (!inRecoveryMode) {
        inRecoveryMode = true;
        recoveryStartTime = currentTime;
      }

      // ===== THROTTLE RESET =====
      // Reset throttle when line is lost (after the 100ms grace period)
      if (wasOnLine) {
        currentThrottle = MIN_THROTTLE;  // Reset to base speed
        wasOnLine = false;               // Mark as off-line
        updateAdaptivePID();             // Update PID values for reset throttle
      }

      // Check if we should attempt recovery based on navigation data or edge sensor memory
      unsigned long timeSinceLoss = currentTime - lineDetectedTime;

      // Always request navigation data when in recovery mode
      requestNavigationData();
      
      // If we've been lost for more than 500ms, request checkpoint return
      if (timeSinceLoss > 500 && timeSinceLoss < 1000) {
        requestCheckpointReturn();
      }

      // First try navigation-based recovery if we have checkpoint data
      if (navData.hasCheckpoint && (currentTime - navData.lastUpdate) < 2000) {
        // Use dead reckoning navigation to return to checkpoint
        executeNavigationCommand();
      } else if (timeSinceLoss < 2000) {  // Within 2000ms window for traditional recovery
        // Enhanced edge sensor memory recovery for curved array
        // Curved array provides better edge detection, so we can extend the memory window
        bool recentLeftEdge = leftEdgeDetected && (currentTime - leftEdgeTime) < 500;  // Extended from 300ms
        bool recentRightEdge = rightEdgeDetected && (currentTime - rightEdgeTime) < 500;  // Extended from 300ms

        if (recentLeftEdge && !recentRightEdge) {
          // Left edge was detected recently - immediate sharp left turn
          sharpLeftTurn();
          lastTurnDirection = -1;
        } else if (recentRightEdge && !recentLeftEdge) {
          // Right edge was detected recently - immediate sharp right turn
          sharpRightTurn();
          lastTurnDirection = 1;
        } else if (lastTurnDirection == -1) {
          // Continue left turn with forward motion for faster search
          if (timeSinceLoss < 500) {
            sharpLeftTurn();  // First 500ms: sharp turn
          } else {
            fastLeftSearch();  // After 500ms: search while moving forward
          }
        } else if (lastTurnDirection == 1) {
          // Continue right turn with forward motion for faster search
          if (timeSinceLoss < 500) {
            sharpRightTurn();  // First 500ms: sharp turn
          } else {
            fastRightSearch();  // After 500ms: search while moving forward
          }
        } else {
          // No memory - aggressive search pattern
          if (timeSinceLoss < 300) {
            sharpLeftTurn();  // Try left first
          } else if (timeSinceLoss < 600) {
            sharpRightTurn();  // Then try right
          } else {
            fastLeftSearch();  // Then search left while moving
          }
        }
      } else {
        // Recovery timeout - stop and reset
        stopMotors();
        integral = 0;
        currentPosition = CURVED_ARRAY_CENTER;  // Reset to center position for curved array
        leftEdgeDetected = false;
        rightEdgeDetected = false;
      }
    }
  }
}

// ===== CURVED ARRAY OPTIMIZATION FUNCTIONS =====
void calculateCurvedArrayParameters() {
  // Calculate optimal parameters based on curved array geometry
  // This function can be called during setup or when parameters need updating
  
  // Adjust error deadband based on curved array precision
  // Curved array provides better precision, so we can use smaller deadband
  ERROR_DEADBAND = 8;  // Reduced from 10 for better curved array response
  
  // Adjust max correction based on curved array's enhanced turn detection
  MAX_CORRECTION = 450;  // Increased from 400 for better curved array control
  
  // Adjust throttle parameters for curved array performance
  THROTTLE_INCREMENT = 12;  // Increased from 10 for faster acceleration with curved array
  THROTTLE_INTERVAL = 180;  // Reduced from 200ms for more responsive curved array control
}

// ===== DOTTED LINE DETECTION FUNCTION =====
bool detectDottedLinePattern() {
  // This function identifies the characteristic pattern of dotted lines:
  // Line detected → Brief white space → Line detected again
  
  // Check if we recently had line detection (within last 200ms)
  // This indicates we just lost the line after having it, which is typical of dotted lines
  unsigned long currentTime = millis();
  bool recentlyHadLine = (currentTime - lastLineDetectionTime) < 200;
  
  // If we had line recently and now lost it, this might be a dotted line
  // The 200ms window is chosen because dotted line gaps are typically short
  if (recentlyHadLine && !lineDetected) {
    return true;  // This looks like a dotted line pattern
  }
  
  return false;  // Not a dotted line pattern
}

// ===== FORWARD-LOOKING LINE DETECTION =====
void handleDottedLineMode() {
  // This function manages the robot's behavior when traversing dotted lines
  // Instead of immediately turning when line is lost, it continues forward
  // to check if there's a line ahead (which is typical of dotted lines)
  
  unsigned long currentTime = millis();
  
  // STEP 1: Check if we should enter dotted line mode
  // Only enter if we're not already in it and we detect a dotted line pattern
  if (!inDottedLineMode && detectDottedLinePattern()) {
    inDottedLineMode = true;                    // Set the mode flag
    dottedLineStartTime = currentTime;          // Record when we entered this mode
    dottedLineForwardCount = 0;                 // Reset the forward step counter
    Serial1.println("Entering dotted line mode - continuing forward");
  }
  
  // STEP 2: Handle behavior while in dotted line mode
  if (inDottedLineMode) {
    // EXIT CONDITION 1: Check if we found line again
    // This is the success case - we found the line after the gap
    if (lineDetected) {
      inDottedLineMode = false;                 // Exit dotted line mode
      Serial1.println("Line found - exiting dotted line mode");
      return;                                   // Skip normal recovery mode
    }
    
    // EXIT CONDITION 2: Check if we've been in dotted line mode too long
    // This prevents the robot from going too far if there's no line ahead
    if ((currentTime - dottedLineStartTime) > DOTTED_LINE_TIMEOUT) {
      inDottedLineMode = false;                 // Exit dotted line mode
      Serial1.println("Dotted line timeout - entering recovery mode");
      return;                                   // Fall back to normal recovery
    }
    
    // EXIT CONDITION 3: Check if we've moved forward enough steps
    // This prevents the robot from going too far off course
    if (dottedLineForwardCount >= MAX_DOTTED_FORWARD_STEPS) {
      inDottedLineMode = false;                 // Exit dotted line mode
      Serial1.println("Max forward steps reached - entering recovery mode");
      return;                                   // Fall back to normal recovery
    }
    
    // STEP 3: Continue forward movement in dotted line mode
    dottedLineForwardCount++;                   // Increment the step counter
    
    // Use last known position to maintain direction
    // This keeps the robot moving in the same direction it was going before losing the line
    float lastKnownError = CURVED_ARRAY_CENTER - currentPosition;
    float correction = constrain(lastKnownError * 0.5, -200, 200);  // Reduced correction (50% of normal)
    
    // Set forward speed slightly above minimum for controlled movement
    int forwardSpeed = MIN_THROTTLE + 20;       // Base speed + 20 for steady forward motion
    int leftSpeed = forwardSpeed - correction;   // Apply correction to left motor
    int rightSpeed = forwardSpeed + correction;  // Apply correction to right motor
    
    // Apply motor speeds with safety constraints
    leftMotor(constrain(leftSpeed, 0, 255));    // Ensure speed is within valid range
    rightMotor(constrain(rightSpeed, 0, 255));  // Ensure speed is within valid range
    
    return;  // Skip normal recovery mode - we're handling this ourselves
  }
}

void setup() {
  analogReadResolution(12);

  // Initialize sensor pins for curved 7-sensor array
  for (int i = 0; i < 7; i++) pinMode(sensorPins[i], INPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // Turn LED on to indicate ready

  // Initialize Serial1 for ESP8266 communication
  Serial1.begin(115200);
  
  // Initialize parameters
  initializeParameters();
  
  // Calculate curved array specific parameters
  calculateCurvedArrayParameters();
  
  // Initialize adaptive PID values
  updateAdaptivePID();
  
  // Brief startup delay
  delay(1000);
}

void motorTest() {
  // Test sequence to verify motor functionality

  // 1. Test left motor forward (500ms)
  leftMotor(150);
  rightMotor(0);
  delay(500);

  // 2. Test right motor forward (500ms)
  leftMotor(0);
  rightMotor(150);
  delay(500);

  // 3. Test both motors forward (500ms)
  leftMotor(150);
  rightMotor(150);
  delay(500);

  // 4. Test left turn (sharp) - 500ms
  leftMotor(-150);
  rightMotor(150);
  delay(500);

  // 5. Test right turn (sharp) - 500ms
  leftMotor(150);
  rightMotor(-150);
  delay(500);

  // 6. Stop and prepare for line following
  stopMotors();
  delay(500);
}

void loop() {
  // Handle serial communication (non-blocking)
  handleSerialCommunication();
  
  // Execute main line following logic
  executeLineFollowing();
  
  // Send status to ESP8266 (non-blocking, time-based)
  sendStatusToESP8266();
  
  // Small delay to prevent overwhelming the system
}
