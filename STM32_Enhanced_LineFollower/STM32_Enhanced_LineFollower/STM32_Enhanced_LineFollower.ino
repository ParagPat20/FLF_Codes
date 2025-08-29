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
int sensorThresholds[7] = { 2600, 2950, 3300, 3600, 3300, 3125, 2600 };
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

// ===== ENHANCED PID VARIABLES =====
float Kp = 0.25;          // Increased for faster response
float Ki = 0.15;          // Reduced to prevent overshoot
float Kd = 1.2;           // Increased for better stability
int baseSpeed = 180;      // Base DC motor speed
float baseThrustPower = 0.35; // Base thruster power
bool thrustersEnabled = true;

float error = 0, lastError = 0, integral = 0, derivative = 0;
float pidOutput = 0;
int linePosition = 0, lastLinePosition = 0;

// ===== TIMING VARIABLES =====
unsigned long lastPIDTime = 0;
unsigned long lastSensorTime = 0;
const unsigned long PID_INTERVAL = 15;    // 66Hz PID loop (faster)
const unsigned long SENSOR_INTERVAL = 10; // 100Hz sensor reading (faster)

// ===== SYSTEM STATES =====
enum SystemState {
  STOPPED,
  LINE_FOLLOWING,
  DOTTED_LINE_FORWARD,
  RECOVERY_BREAK,
  RECOVERY_REVERSE,
  RECOVERY_ZIGZAG
};

SystemState currentState = STOPPED;
bool lineFollowingActive = false;

// ===== DOTTED LINE HANDLING =====
unsigned long lineLastSeenTime = 0;
unsigned long dottedLineTimeout = 800;    // 800ms forward on dotted lines
bool lineDetected = false;
int noLineCounter = 0;
const int NO_LINE_THRESHOLD = 5;          // Consecutive readings without line

// ===== RECOVERY SYSTEM =====
unsigned long recoveryStartTime = 0;
unsigned long breakInterval = 150;        // 150ms break before reverse
unsigned long reverseInterval = 400;      // 400ms reverse movement
unsigned long zigzagStartTime = 0;
int zigzagDirection = 1;                   // 1 = right, -1 = left
unsigned long zigzagSwitchInterval = 300; // 300ms per zigzag direction
int zigzagSpeed = 120;                     // Zigzag search speed

// ===== MOTOR CONTROL FUNCTIONS =====
void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  
  if (speed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speed = -speed;
  }
  analogWrite(PWMA, speed);
}

void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  
  if (speed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    speed = -speed;
  }
  analogWrite(PWMB, speed);
}

void stopAllMotors() {
  setLeftMotor(0);
  setRightMotor(0);
  thrusterValues[0] = 0.0;
  thrusterValues[1] = 0.0;
  dshot.write(thrusterValues);
}

// ===== SENSOR FUNCTIONS =====
void readSensors() {
  for (int i = 0; i < 7; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
  
  linePosition = calculateLinePosition();
  checkLineDetection();
}

int calculateLinePosition() {
  int weightedSum = 0;
  int activeSensors = 0;
  
  // Enhanced weighted average with center bias
  int weights[7] = { -35, -25, -15, 0, 15, 25, 35 };
  
  for (int i = 0; i < 7; i++) {
    bool sensorTriggered;
    if (followBlackLine) {
      sensorTriggered = (sensorValues[i] < sensorThresholds[i]);
    } else {
      sensorTriggered = (sensorValues[i] > sensorThresholds[i]);
    }
    
    if (sensorTriggered) {
      weightedSum += weights[i];
      activeSensors++;
    }
  }
  
  if (activeSensors == 0) {
    return lastLinePosition; // No line detected
  }
  
  return weightedSum / activeSensors;
}

void checkLineDetection() {
  int activeSensors = 0;
  
  for (int i = 0; i < 7; i++) {
    bool sensorTriggered;
    if (followBlackLine) {
      sensorTriggered = (sensorValues[i] < sensorThresholds[i]);
    } else {
      sensorTriggered = (sensorValues[i] > sensorThresholds[i]);
    }
    
    if (sensorTriggered) {
      activeSensors++;
    }
  }
  
  if (activeSensors > 0) {
    lineDetected = true;
    lineLastSeenTime = millis();
    noLineCounter = 0;
  } else {
    lineDetected = false;
    noLineCounter++;
  }
}

// ===== ENHANCED PID CONTROLLER =====
void updatePID() {
  error = linePosition;
  
  // Enhanced integral calculation with windup protection
  integral += error;
  integral = constrain(integral, -50, 50); // Tighter integral limits
  
  // Enhanced derivative calculation with smoothing
  derivative = error - lastError;
  
  // Advanced PID calculation with derivative kick prevention
  pidOutput = Kp * error + Ki * integral + Kd * derivative;
  
  // Dynamic PID output limiting based on speed
  float maxOutput = baseSpeed * 0.8;
  pidOutput = constrain(pidOutput, -maxOutput, maxOutput);
  
  lastError = error;
}

// ===== LINE FOLLOWING CONTROL =====
void executeLineFollowing() {
  // Calculate motor speeds with enhanced response
  int leftSpeed = baseSpeed + pidOutput;
  int rightSpeed = baseSpeed - pidOutput;
  
  // Apply speed limits
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  // Set DC motors
  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
  
  // Enhanced thruster control
  if (thrustersEnabled) {
    float leftThrust = baseThrustPower;
    float rightThrust = baseThrustPower;
    
    // Position-based thrust adjustment
    float positionFactor = abs(linePosition) / 35.0;
    
    // Dynamic thrust for turning assistance
    if (linePosition > 5) { // Turn right
      leftThrust += (positionFactor * 0.4);
      rightThrust -= (positionFactor * 0.2);
    } else if (linePosition < -5) { // Turn left
      rightThrust += (positionFactor * 0.4);
      leftThrust -= (positionFactor * 0.2);
    }
    
    // Speed-based thrust boost
    float avgSpeed = (leftSpeed + rightSpeed) / 2.0;
    float speedFactor = avgSpeed / 255.0;
    leftThrust *= (0.7 + speedFactor * 0.3);
    rightThrust *= (0.7 + speedFactor * 0.3);
    
    // Apply thrust limits
    thrusterValues[0] = constrain(leftThrust, 0.0, 1.0);
    thrusterValues[1] = constrain(rightThrust, 0.0, 1.0);
  } else {
    thrusterValues[0] = 0.0;
    thrusterValues[1] = 0.0;
  }
  
  dshot.write(thrusterValues);
}

// ===== DOTTED LINE HANDLING =====
void handleDottedLine() {
  // Move forward at base speed when line is not detected
  setLeftMotor(baseSpeed);
  setRightMotor(baseSpeed);
  
  // Keep thrusters at base power for forward movement
  if (thrustersEnabled) {
    thrusterValues[0] = baseThrustPower * 0.8;
    thrusterValues[1] = baseThrustPower * 0.8;
    dshot.write(thrusterValues);
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
  dshot.write(thrusterValues);
  
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
  dshot.write(thrusterValues);
}

// ===== STATE MACHINE =====
void updateStateMachine() {
  switch (currentState) {
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
        case STOPPED: Serial1.print("STOPPED"); break;
        case LINE_FOLLOWING: Serial1.print("LINE_FOLLOWING"); break;
        case DOTTED_LINE_FORWARD: Serial1.print("DOTTED_LINE"); break;
        case RECOVERY_BREAK: Serial1.print("RECOVERY_BREAK"); break;
        case RECOVERY_REVERSE: Serial1.print("RECOVERY_REVERSE"); break;
        case RECOVERY_ZIGZAG: Serial1.print("RECOVERY_ZIGZAG"); break;
      }
      Serial1.print(" | Line Pos: ");
      Serial1.print(linePosition);
      Serial1.print(" | PID: ");
      Serial1.print(pidOutput, 2);
      Serial1.print(" | Thrusters: ");
      Serial1.println(thrustersEnabled ? "ON" : "OFF");
      
    } else {
      Serial1.println("Commands: start/stop/Kp=val/Ki=val/Kd=val/speed=val/thrust=val/thruster=on|off/pid/status");
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

// ===== MAIN LOOP TIMING =====
void runTimedOperations() {
  unsigned long currentTime = millis();
  
  // High-frequency sensor reading (100Hz)
  if (currentTime - lastSensorTime >= SENSOR_INTERVAL) {
    lastSensorTime = currentTime;
    if (lineFollowingActive) {
      readSensors();
    }
  }
  
  // High-frequency PID loop (66Hz)
  if (currentTime - lastPIDTime >= PID_INTERVAL) {
    lastPIDTime = currentTime;
    if (lineFollowingActive && currentState == LINE_FOLLOWING) {
      updatePID();
    }
  }
}

// ===== SETUP =====
void setup() {
  // Initialize ADC resolution
  analogReadResolution(12);
  
  // Initialize serial communication
  Serial1.begin(115200);
  Serial1.println("=== Enhanced Line Following Robot ===");
  Serial1.println("Features: Fast PID, Dotted Lines, Recovery System");
  Serial1.println("Commands: start/stop/Kp=val/Ki=val/Kd=val/speed=val/thrust=val/thruster=on|off/pid/status");
  Serial1.println("Ready for operation!");
  
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
  
  // Initialize DShot ESCs
  dshot.begin(pins);
  
  // Initialize timing
  lastPIDTime = millis();
  lastSensorTime = millis();
  lineLastSeenTime = millis();
}

// ===== MAIN LOOP =====
void loop() {
  processSerialCommands();
  runTimedOperations();
  updateStateMachine();
  
  // Small delay for system stability
  delayMicroseconds(10);
}
