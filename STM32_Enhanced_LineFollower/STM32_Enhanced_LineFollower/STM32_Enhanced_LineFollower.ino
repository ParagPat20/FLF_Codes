/*
  Enhanced Line Following Robot with Advanced PID Control
  STM32F405 + DShot ESCs + TB6612 DC Motors + 7-Sensor Array
  
  Features:
  - Fast and responsive PID controller
  - Dotted line handling with forward movement
  - Advanced recovery system with break, reverse, and zigzag search
  - Thruster control (ESCs) with on/off capability
  - Serial commands for real-time PID tuning
  
  Serial Commands:
  - "start"         → begin line following
  - "stop"          → stop all motors and thrusters
  - "Kp=val"        → set Kp value
  - "Ki=val"        → set Ki value  
  - "Kd=val"        → set Kd value
  - "speed=val"     → set base speed (0-255)
  - "thrust=val"    → set base thruster power (0.0-1.0)
  - "thruster=on/off" → enable/disable thrusters
  - "pid"           → show current PID values
  - "status"        → show system status
*/

#include <dshot_stm32f4.h>
#include <HardwareSerial.h>
#include <vector>

// ===== SENSOR CONFIGURATION =====
const uint8_t sensorPins[7] = { PA4, PA5, PA6, PA0, PA1, PA2, PA3 };
int sensorThresholds[7] = { 2550, 2700, 2850, 3300, 3150, 2850, 2450 };
int sensorValues[7];
bool followBlackLine = true;

// ===== HARDWARE PINS =====
HardwareSerial Serial1(USART1);

// ESC pins (Thrusters)
static const uint8_t PIN1 = PB8;  // Left Thruster
static const uint8_t PIN2 = PB9;  // Right Thruster

// DC Motor driver pins (TB6612)
#define PWMA PB7
#define AIN2 PB14
#define AIN1 PB15
#define BIN1 PA8
#define BIN2 PB3
#define PWMB PB4

// Status LED
#define LED_PIN PC13

// ===== DSHOT SETUP =====
static std::vector<uint8_t> pins = { PIN1, PIN2 };
static Stm32F4Dshot dshot;
static float thrusterValues[2] = { 0.0, 0.0 };

// ===== PID VARIABLES (match TB6612 main) =====
float Kp = 0.15;
float Ki = 0.030;
float Kd = 0.08;
int baseSpeed = 140;
int currentSpeed = 140;
float error = 0, lastError = 0, integral = 0;

// ===== TIMING (match TB6612 main) =====
static const uint32_t UPDATE_RATE = 1000;  // 1 kHz control loop
unsigned long lastPidTime = 0;
unsigned long lastSampleTime = 0;
const unsigned long PID_INTERVAL = 1;
const unsigned long SENSOR_INTERVAL = 1;

// ===== SYSTEM STATES =====
// ===== ROBOT STATE/BOOT =====
unsigned long bootTime = 0;

// ===== MOTOR CONTROL FUNCTIONS =====
void setLeftMotor(int speed) {
  
  if (speed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speed = -speed;
  }
  // Bound PWM only (allow full -255..255 range prior to this point)
  analogWrite(PWMA, constrain(speed, 0, 255));
}

void setRightMotor(int speed) {
  
  if (speed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    speed = -speed;
  }
  // Bound PWM only (allow full -255..255 range prior to this point)
  analogWrite(PWMB, constrain(speed, 0, 255));
}

void stopAllMotors() { setLeftMotor(0); setRightMotor(0); thrusterValues[0]=0.0; thrusterValues[1]=0.0; }

// ===== CURVED SENSOR + 2D ERROR (match TB6612 main) =====
struct PoseError { float e_lat_mm; float e_yaw_rad; bool valid; };
struct SensorGeometry { float distance; float angle; float weight; float position; };
const SensorGeometry sensorGeometry[7] = {
  { 46.5, 99.27, 6000, 0.0 }, { 23.019, 58.61, 5000, 1000.0 }, { 23.019, 40.66, 4000, 2000.0 },
  { 0.0, 0.0, 3000, 3000.0 }, { 21.76, -36.69, 2000, 4000.0 }, { 40.966, -72.07, 1000, 5000.0 }, { 64.0, -111.4, 0, 6000.0 }
};
#define CURVED_WEIGHT_MULTIPLIER 1.0
#define CURVED_ARRAY_CENTER 3000.0
#define LOOKAHEAD_DISTANCE 80.0
#define CURVATURE_FACTOR 2.5
#define MIN_SPEED_FACTOR 0.4
#define MAX_SPEED_FACTOR 1.0
int MAX_CORRECTION = 400;
int ERROR_DEADBAND = 10;
bool lineDetected = false;
unsigned long lineDetectedTime = 0;
float currentPosition = 3000;
bool inDottedLineMode = false;
unsigned long dottedLineStartTime = 0;
unsigned long lastLineDetectionTime = 0;
int dottedLineForwardCount = 0;
const int MAX_DOTTED_FORWARD_STEPS = 40;
const unsigned long DOTTED_LINE_TIMEOUT = 800;
bool leftEdgeDetected = false;
bool rightEdgeDetected = false;
unsigned long leftEdgeTime = 0;
unsigned long rightEdgeTime = 0;
int lastTurnDirection = 0;
bool inRecoveryMode = false;
unsigned long recoveryStartTime = 0;

PoseError calculate2DError() {
  PoseError result = { 0.0, 0.0, false };
  float totalWeight = 0, weightedX = 0, weightedY = 0; int activeCount = 0;
  for (int i = 0; i < 7; i++) {
    int value = analogRead(sensorPins[i]);
    if (value < sensorThresholds[i]) {
      activeCount++;
      float angleRad = sensorGeometry[i].angle * PI / 180.0;
      float x = sensorGeometry[i].distance * cos(angleRad);
      float y = sensorGeometry[i].distance * sin(angleRad);
      float weight = sensorGeometry[i].weight;
      if (i == 0 || i == 6) weight *= CURVED_WEIGHT_MULTIPLIER * 1.5;
      else if (i == 1 || i == 5) weight *= CURVED_WEIGHT_MULTIPLIER * 1.2;
      else if (i == 2 || i == 4) weight *= CURVED_WEIGHT_MULTIPLIER;
      weightedX += x * weight; weightedY += y * weight; totalWeight += weight;
      if (i == 0 || i == 1) { leftEdgeDetected = true; leftEdgeTime = millis(); }
      if (i == 5 || i == 6) { rightEdgeDetected = true; rightEdgeTime = millis(); }
    }
  }
  if (activeCount > 0 && totalWeight > 0) {
    float centroidX = weightedX / totalWeight; float centroidY = weightedY / totalWeight;
    result.e_lat_mm = -centroidY;
    result.e_yaw_rad = atan2(centroidY, centroidX + 50.0);
    result.valid = true;
  }
  return result;
}

float readLinePosition() {
  PoseError pe = calculate2DError();
  if (pe.valid) { float position = CURVED_ARRAY_CENTER - (pe.e_lat_mm * 10.0); return constrain(position, 0, 6000); }
  return -1;
}

void updateAdaptiveSpeed(float headingError) {
  float absHeadingError = fabs(headingError);
  float speedFactor = 1.0f / (1.0f + CURVATURE_FACTOR * absHeadingError);
  speedFactor = constrain(speedFactor, MIN_SPEED_FACTOR, MAX_SPEED_FACTOR);
  currentSpeed = (int)(baseSpeed * speedFactor);
}

float calculateMixedError(const PoseError& pe) { return pe.e_lat_mm + (LOOKAHEAD_DISTANCE * pe.e_yaw_rad * 180.0 / PI); }

// PID is computed inline in executeLineFollowing using TB6612 pattern

// ===== MAIN LINE FOLLOWING LOGIC (TB6612) =====
void executeLineFollowing() {
  unsigned long currentTime = millis();
  PoseError poseError = { 0.0, 0.0, false };
  if (currentTime - lastSampleTime >= SENSOR_INTERVAL) {
    lastSampleTime = currentTime;
    poseError = calculate2DError();
    if (poseError.valid) {
      currentPosition = CURVED_ARRAY_CENTER - (poseError.e_lat_mm * 10.0);
      currentPosition = constrain(currentPosition, 0, 6000);
      lineDetected = true; lineDetectedTime = currentTime; lastLineDetectionTime = currentTime;
    } else { lineDetected = false; }
  }
  if (currentTime - lastPidTime >= PID_INTERVAL) {
    lastPidTime = currentTime;
    if (lineDetected && (currentTime - lineDetectedTime) < 100) {
      inRecoveryMode = false;
      if (poseError.valid) {
        error = calculateMixedError(poseError);
        updateAdaptiveSpeed(poseError.e_yaw_rad);
        if (abs(error) < ERROR_DEADBAND) { error = 0; integral = 0; }
        if (poseError.e_lat_mm < -20.0) lastTurnDirection = -1; else if (poseError.e_lat_mm > 20.0) lastTurnDirection = 1;
      } else { error = CURVED_ARRAY_CENTER - currentPosition; currentSpeed = baseSpeed; }
      float deltaTime = PID_INTERVAL / 1000.0f; integral += error * deltaTime; integral = constrain(integral, -500, 500);
      float derivative = (error - lastError) / deltaTime; float rawCorrection = Kp * error + Ki * integral + Kd * derivative;
      float correction = constrain(rawCorrection, -MAX_CORRECTION, MAX_CORRECTION); lastError = error;
      int leftSpeed = currentSpeed - correction; int rightSpeed = currentSpeed + correction;
      // ESC throttle update based on error magnitude (match TB6612 style)
      float errorMagnitude = abs(error);
      if (errorMagnitude < ERROR_DEADBAND) { thrusterValues[0] = 0.5; thrusterValues[1] = 0.5; }
      else if (errorMagnitude < 100) { float baseT = 0.6f; float adj = correction / 1200.0f; thrusterValues[0] = constrain(baseT - adj, 0.4, 0.9); thrusterValues[1] = constrain(baseT + adj, 0.4, 0.9); }
      else { float baseT = 0.7f; float adj = correction / 1000.0f; thrusterValues[0] = constrain(baseT - adj, 0.4, 0.9); thrusterValues[1] = constrain(baseT + adj, 0.4, 0.9); }
      setLeftMotor(constrain(leftSpeed, -255, 255)); setRightMotor(constrain(rightSpeed, -255, 255));
    } else {
      // Dotted line handling + recovery (TB6612)
      // Enter dotted line mode check
      // Reuse helper below
      handleDottedLineMode();
      if (inDottedLineMode) return;
      if (!inRecoveryMode) { inRecoveryMode = true; recoveryStartTime = currentTime; }
      unsigned long timeSinceLoss = currentTime - lineDetectedTime;
      if (timeSinceLoss < 2000) {
        bool recentLeft = leftEdgeDetected && (currentTime - leftEdgeTime) < 500;
        bool recentRight = rightEdgeDetected && (currentTime - rightEdgeTime) < 500;
        if (recentLeft && !recentRight) { leftMotor(-200); rightMotor(200); lastTurnDirection = -1; }
        else if (recentRight && !recentLeft) { leftMotor(200); rightMotor(-200); lastTurnDirection = 1; }
        else if (lastTurnDirection == -1) { if (timeSinceLoss < 500) { leftMotor(-200); rightMotor(200); } else { leftMotor(50); rightMotor(200); } }
        else if (lastTurnDirection == 1) { if (timeSinceLoss < 500) { leftMotor(200); rightMotor(-200); } else { leftMotor(200); rightMotor(50); } }
        else { if (timeSinceLoss < 300) { leftMotor(-200); rightMotor(200); } else if (timeSinceLoss < 600) { leftMotor(200); rightMotor(-200); } else { leftMotor(200); rightMotor(50); } }
      } else { thrusterValues[0]=0.0; thrusterValues[1]=0.0; stopAllMotors(); integral = 0; currentPosition = CURVED_ARRAY_CENTER; leftEdgeDetected=false; rightEdgeDetected=false; }
    }
  }
}

// ===== DOTTED LINE DETECTION =====
bool detectDottedLinePattern() { unsigned long t=millis(); bool recentlyHadLine=(t-lastLineDetectionTime)<200; return (recentlyHadLine && !lineDetected); }
void handleDottedLineMode() {
  unsigned long t=millis(); if (!inDottedLineMode && detectDottedLinePattern()) { inDottedLineMode=true; dottedLineStartTime=t; dottedLineForwardCount=0; }
  if (inDottedLineMode) {
    if (lineDetected) { inDottedLineMode=false; return; }
    if ((t - dottedLineStartTime) > DOTTED_LINE_TIMEOUT) { inDottedLineMode=false; return; }
    if (dottedLineForwardCount >= MAX_DOTTED_FORWARD_STEPS) { inDottedLineMode=false; return; }
    dottedLineForwardCount++;
    float lastKnownError = CURVED_ARRAY_CENTER - currentPosition; float correction = constrain(lastKnownError * 0.5, -200, 200);
    thrusterValues[0] = 0.7; thrusterValues[1] = 0.7;
    int forwardSpeed = baseSpeed + 20; int l = forwardSpeed - correction; int r = forwardSpeed + correction;
    setLeftMotor(constrain(l, 0, 255)); setRightMotor(constrain(r, 0, 255));
    return;
  }
}

// ===== RECOVERY SYSTEM =====
void executeRecoveryBreak() {
  // Stop all motors during break
  stopAllMotors();
  
  if (millis() - recoveryStartTime > breakInterval) {
    currentState = RECOVERY_REVERSE;
    recoveryStartTime = millis();
    Serial1.println("Recovery: Starting reverse");
  }
}

void executeRecoveryReverse() {
  // Move backwards to find line
  setLeftMotor(-100);
  setRightMotor(-100);
  
  // Thrusters off during recovery
  thrusterValues[0] = 0.0;
  thrusterValues[1] = 0.0;
  // Note: dshot.write() called in main loop timing function
  
  if (millis() - recoveryStartTime > reverseInterval) {
    currentState = RECOVERY_ZIGZAG;
    zigzagStartTime = millis();
    zigzagDirection = 1;
    Serial1.println("Recovery: Starting zigzag search");
  }
}

void executeRecoveryZigzag() {
  // Zigzag pattern to find line
  if (millis() - zigzagStartTime > zigzagSwitchInterval) {
    zigzagDirection *= -1; // Switch direction
    zigzagStartTime = millis();
  }
  
  // Apply zigzag movement
  if (zigzagDirection > 0) {
    setLeftMotor(zigzagSpeed);
    setRightMotor(zigzagSpeed * 0.3);
  } else {
    setLeftMotor(zigzagSpeed * 0.3);
    setRightMotor(zigzagSpeed);
  }
  
  // Thrusters off during recovery
  thrusterValues[0] = 0.0;
  thrusterValues[1] = 0.0;
  // Note: dshot.write() called in main loop timing function
}

// ===== STATE MACHINE =====
void updateStateMachine() {
  switch (currentState) {
    case ESC_CALIBRATION:
      if (millis() - bootTime > ESC_CALIBRATION_TIME) {
        currentState = LINE_FOLLOWING;
        lineFollowingActive = true;
        Serial1.println("ESC Calibration complete. Starting line following.");
      }
      break;
      
    case STOPPED:
      stopAllMotors();
      break;
      
    case LINE_FOLLOWING:
      if (lineDetected) {
        executeLineFollowing();
      } else if (noLineCounter > NO_LINE_THRESHOLD) {
        // Switch to dotted line mode
        currentState = DOTTED_LINE_FORWARD;
        Serial1.println("Dotted line detected - moving forward");
      }
      break;
      
    case DOTTED_LINE_FORWARD:
      if (lineDetected) {
        // Line found again, return to following
        currentState = LINE_FOLLOWING;
        Serial1.println("Line found - resuming following");
      } else if (millis() - lineLastSeenTime > dottedLineTimeout) {
        // Timeout reached, enter recovery
        currentState = RECOVERY_BREAK;
        recoveryStartTime = millis();
        Serial1.println("Recovery: Starting break phase");
      } else {
        handleDottedLine();
      }
      break;
      
    case RECOVERY_BREAK:
      if (lineDetected) {
        currentState = LINE_FOLLOWING;
        Serial1.println("Line found during break - resuming following");
      } else {
        executeRecoveryBreak();
      }
      break;
      
    case RECOVERY_REVERSE:
      if (lineDetected) {
        currentState = LINE_FOLLOWING;
        Serial1.println("Line found during reverse - resuming following");
      } else {
        executeRecoveryReverse();
      }
      break;
      
    case RECOVERY_ZIGZAG:
      if (lineDetected) {
        currentState = LINE_FOLLOWING;
        Serial1.println("Line found during zigzag - resuming following");
      } else {
        executeRecoveryZigzag();
      }
      break;

    case MANUAL:
      // Thrusters off in manual mode
      thrusterValues[0] = 0.0;
      thrusterValues[1] = 0.0;
      setLeftMotor(manualLeftSpeed);
      setRightMotor(manualRightSpeed);
      break;
  }
}

// ===== SERIAL COMMAND PROCESSING =====
void processSerialCommands() {
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim();
    
    Serial1.print("Command: ");
    Serial1.println(cmd);
    
    if (cmd.equalsIgnoreCase("start")) {
      currentState = LINE_FOLLOWING;
      lineFollowingActive = true;
      Serial1.println("Line following STARTED");
      
    } else if (cmd.equalsIgnoreCase("stop")) {
      currentState = STOPPED;
      lineFollowingActive = false;
      stopAllMotors();
      Serial1.println("System STOPPED");

    } else if (cmd.equalsIgnoreCase("manual")) {
      currentState = MANUAL;
      lineFollowingActive = false;
      thrustersEnabled = false;
      manualLeftSpeed = 0;
      manualRightSpeed = 0;
      Serial1.println("Manual mode ENABLED");

    } else if (cmd.equalsIgnoreCase("auto")) {
      currentState = LINE_FOLLOWING;
      lineFollowingActive = true;
      Serial1.println("Auto line following ENABLED");
      
    } else if (cmd.startsWith("LM=")) {
      int val = constrain(cmd.substring(3).toInt(), -255, 255);
      manualLeftSpeed = val;
      if (currentState == MANUAL) setLeftMotor(manualLeftSpeed);
      Serial1.print("Left DC motor set to: ");
      Serial1.println(val);

    } else if (cmd.startsWith("RM=")) {
      int val = constrain(cmd.substring(3).toInt(), -255, 255);
      manualRightSpeed = val;
      if (currentState == MANUAL) setRightMotor(manualRightSpeed);
      Serial1.print("Right DC motor set to: ");
      Serial1.println(val);

    } else if (cmd.startsWith("bothdc=")) {
      int val = constrain(cmd.substring(7).toInt(), -255, 255);
      manualLeftSpeed = val;
      manualRightSpeed = val;
      if (currentState == MANUAL) {
        setLeftMotor(manualLeftSpeed);
        setRightMotor(manualRightSpeed);
      }
      Serial1.print("Both DC motors set to: ");
      Serial1.println(val);

    } else if (cmd.equalsIgnoreCase("stopdc")) {
      manualLeftSpeed = 0;
      manualRightSpeed = 0;
      setLeftMotor(0);
      setRightMotor(0);
      Serial1.println("Both DC motors STOPPED");
      
    } else if (cmd.startsWith("Kp=")) {
      Kp = cmd.substring(3).toFloat();
      Serial1.print("Kp set to: ");
      Serial1.println(Kp, 3);
      
    } else if (cmd.startsWith("Ki=")) {
      Ki = cmd.substring(3).toFloat();
      Serial1.print("Ki set to: ");
      Serial1.println(Ki, 3);
      
    } else if (cmd.startsWith("Kd=")) {
      Kd = cmd.substring(3).toFloat();
      Serial1.print("Kd set to: ");
      Serial1.println(Kd, 3);
      
    } else if (cmd.startsWith("speed=")) {
      baseSpeed = constrain(cmd.substring(6).toInt(), 0, 255);
      Serial1.print("Base speed set to: ");
      Serial1.println(baseSpeed);
      
    } else if (cmd.startsWith("thrust=")) {
      baseThrustPower = constrain(cmd.substring(7).toFloat(), 0.0, 1.0);
      Serial1.print("Base thrust power set to: ");
      Serial1.println(baseThrustPower, 2);
      
    } else if (cmd.startsWith("thruster=")) {
      String value = cmd.substring(9);
      thrustersEnabled = (value == "on");
      Serial1.print("Thrusters: ");
      Serial1.println(thrustersEnabled ? "ON" : "OFF");
      
    } else if (cmd.equalsIgnoreCase("pid")) {
      Serial1.print("PID Values - Kp: ");
      Serial1.print(Kp, 3);
      Serial1.print(" Ki: ");
      Serial1.print(Ki, 3);
      Serial1.print(" Kd: ");
      Serial1.print(Kd, 3);
      Serial1.print(" Speed: ");
      Serial1.println(baseSpeed);
      
    } else if (cmd.equalsIgnoreCase("status")) {
      Serial1.print("State: ");
      switch (currentState) {
        case ESC_CALIBRATION: Serial1.print("ESC_CALIBRATION"); break;
        case STOPPED: Serial1.print("STOPPED"); break;
        case LINE_FOLLOWING: Serial1.print("LINE_FOLLOWING"); break;
        case DOTTED_LINE_FORWARD: Serial1.print("DOTTED_LINE"); break;
        case RECOVERY_BREAK: Serial1.print("RECOVERY_BREAK"); break;
        case RECOVERY_REVERSE: Serial1.print("RECOVERY_REVERSE"); break;
        case RECOVERY_ZIGZAG: Serial1.print("RECOVERY_ZIGZAG"); break;
        case MANUAL: Serial1.print("MANUAL"); break;
      }
      Serial1.print(" | Line Pos: ");
      Serial1.print(linePosition);
      Serial1.print(" | PID: ");
      Serial1.print(pidOutput, 2);
      Serial1.print(" | Thrusters: ");
      Serial1.println(thrustersEnabled ? "ON" : "OFF");
      
    } else {
      Serial1.println("Commands: start/stop/manual/auto/LM=n/RM=n/bothdc=n/stopdc/Kp=val/Ki=val/Kd=val/speed=val/thrust=val/thruster=on|off/pid/status");
    }
  }
}

// ===== DMA INTERRUPT HANDLERS =====
extern "C" void DMA2_Stream1_IRQHandler(void) {
  dshot.handleDmaIrqStream1();
}

extern "C" void DMA2_Stream2_IRQHandler(void) {
  dshot.handleDmaIrqStream2();
}

// ===== ESC TIMING CONTROL =====
static void runESCTiming(const uint32_t usec) {
  static uint32_t prev;
  const uint32_t UPDATE_RATE = 50;  // 50Hz ESC update rate
  
  if (usec - prev > 1000000 / UPDATE_RATE) {
    prev = usec;
    // Always send ESC commands to maintain DShot communication
    dshot.write(thrusterValues);
  }
}

// ===== CONTROL LOOP GATING (match TB6612 main) =====
static void runControl(const uint32_t usec) {
  static uint32_t prev;
  if (usec - prev > 1000000 / UPDATE_RATE) { prev = usec; executeLineFollowing(); }
}

// ===== SETUP =====
void setup() {
  // Initialize ADC resolution
  analogReadResolution(12);
  
  // Initialize sensor pins
  for (int i = 0; i < 7; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  
  // Initialize motor driver pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  
  // Initialize status LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  dshot.begin(pins);
  bootTime = millis();
}

// ===== MAIN LOOP =====
void loop() {
  const auto usec = micros();
  
  runESCTiming(usec);
  if (millis() - bootTime > 3000) { runControl(usec); }
}
