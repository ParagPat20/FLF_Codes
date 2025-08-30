/*
   Independent motor control for 2 ESCs on PB8 & PB10 (STM32F405 + DShot).
   Line following with PID control using 7-sensor array.

   Serial Commands:
   - "start"        → begin sweep (min→max→min loop, both motors together)
   - "stop"         → stop sweep & motors off
   - "M1=0.3"       → set Motor 1 throttle (0.0–1.0)
   - "M2=0.7"       → set Motor 2 throttle (0.0–1.0)
   - "both=0.5"     → set both motors to same throttle
   - "linefollow"   → start line following mode
   - "stopline"     → stop line following mode
   - "pid"          → show current PID values
   - "Kp=20"        → set Kp value
   - "Ki=0.1"       → set Ki value
   - "Kd=0.5"       → set Kd value
   - "basespeed=150" → set base speed
*/

#include <dshot_stm32f4.h>
#include <HardwareSerial.h>
#include <vector>
const uint8_t sensorPins[7] = { PA4, PA5, PA6, PA0, PA1, PA2, PA3 };
// Dynamic threshold values - can be updated via web interface
int sensorThresholds[7] = { 2550, 2880, 3000, 3500, 3150, 3125, 2600 };
int sensorValues[7];
//ESC pins
HardwareSerial Serial1(USART1);

static const uint8_t PIN1 = PB8;  // Motor 1
static const uint8_t PIN2 = PB9;  // Motor 2

// Motor driver pins
#define PWMA PB7
#define AIN2 PB14
#define AIN1 PB15
#define BIN1 PA8
#define BIN2 PB3
#define PWMB PB4

// LED pin for calibration indication
#define LED_PIN PC13

static std::vector<uint8_t> pins = { PIN1, PIN2 };
static Stm32F4Dshot dshot;

// Motor throttle values [0.0 – 1.0]
static float motorval[2] = { 0.0, 0.0 };

// Modes
static bool sweeping = false;
static bool increasing = true;
static bool lineFollowing = false;

// Sweep settings
static const uint32_t UPDATE_RATE = 50;  // Hz (20 ms)
static const float STEP = 0.01;          // sweep step

// ===== PID VARIABLES =====
float Kp = 0.15;
float Ki = 0.2;
float Kd = 0.8;
int baseSpeed = 200;
float baseThrottle = 0.4; // Base ESC thrust (0.0 to 1.0)

float error = 0, lastError = 0, integral = 0;

// ===== TIMER VARIABLES =====
unsigned long lastPIDTime = 0;
const unsigned long CONTROL_INTERVAL = 20;    // 50Hz control loop (sensors + PID)

// ===== LINE FOLLOWING VARIABLES =====
int linePosition = 0;
int lastLinePosition = 0;
float pidOutput = 0;

// ===== DEBUG VARIABLES =====
bool sensorPrintEnabled = false;  // Toggle for sensor values printing
bool pidStatusEnabled = false;    // Toggle for PID status printing

// ===== SETUP VARIABLES =====
bool followBlackLine = true;      // true = follow black, false = follow white
bool leftMotorInverted = false;   // true = invert left DC motor direction
bool rightMotorInverted = false;  // true = invert right DC motor direction
bool escSwapped = false;          // true = ESC1 acts as right, ESC2 as left
bool esc1Inverted = false;        // true = invert ESC1 direction
bool esc2Inverted = false;        // true = invert ESC2 direction

// ===== MOTOR FUNCTIONS =====
void leftMotor(int speed) {
  // Apply inversion if configured
  if (leftMotorInverted) {
    speed = -speed;
  }
  
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
  // Apply inversion if configured
  if (rightMotorInverted) {
    speed = -speed;
  }
  
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

// ===== LINE FOLLOWING FUNCTIONS =====
void readSensors() {
  for (int i = 0; i < 7; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
  
  // Debug: Print sensor values if enabled
  if (sensorPrintEnabled) {
    printSensorValues();
  }
  
  // Calculate line position based on sensor values
  linePosition = calculateLinePosition();
}

void printSensorValues() {
  Serial1.print("Sensors: ");
  for (int i = 0; i < 7; i++) {
    Serial1.print(sensorValues[i]);
    Serial1.print(" (");
    Serial1.print(sensorThresholds[i]);
    Serial1.print(")");
    if (i < 6) Serial1.print(" | ");
  }
  Serial1.print(" -> Line: ");
  Serial1.println(linePosition);
}

void printPIDStatus(int leftSpeed, int rightSpeed, float leftThrust, float rightThrust, float positionFactor, float pidFactor) {
  Serial1.print("PID Debug - Line Pos: ");
  Serial1.print(linePosition);
  Serial1.print(" | Error: ");
  Serial1.print(error, 2);
  Serial1.print(" | Integral: ");
  Serial1.print(integral, 2);
  Serial1.print(" | PID Out: ");
  Serial1.print(pidOutput, 2);
  Serial1.print(" | DC L: ");
  Serial1.print(leftSpeed);
  Serial1.print(" R: ");
  Serial1.print(rightSpeed);
  Serial1.print(" | ESC L: ");
  Serial1.print(leftThrust, 2);
  Serial1.print(" R: ");
  Serial1.print(rightThrust, 2);
  Serial1.print(" | PosF: ");
  Serial1.print(positionFactor, 2);
  Serial1.print(" PIDF: ");
  Serial1.println(pidFactor, 2);
}

int calculateLinePosition() {
  int weightedSum = 0;
  int sum = 0;
  int activeSensors = 0;
  
  // Weighted average calculation
  // Center sensors have higher weight
  int weights[7] = { -30, -20, -10, 0, 10, 20, 30 };
  
  for (int i = 0; i < 7; i++) {
    bool lineDetected;
    if (followBlackLine) {
      lineDetected = (sensorValues[i] < sensorThresholds[i]); // Black line detection
    } else {
      lineDetected = (sensorValues[i] > sensorThresholds[i]); // White line detection
    }
    
    if (lineDetected) {
      weightedSum += weights[i];
      sum += weights[i];
      activeSensors++;
    }
  }
  
  if (activeSensors == 0) {
    // No line detected, return last known position
    return lastLinePosition;
  }
  
  return weightedSum / activeSensors;
}

void updatePID() {
  // Calculate error (line position)
  error = linePosition;
  
  // PID calculation
  integral += error;
  float derivative = error - lastError;
  
  // Anti-windup for integral term
  integral = constrain(integral, -100, 100);
  
  pidOutput = Kp * error + Ki * integral + Kd * derivative;
  
  // Constrain PID output
  pidOutput = constrain(pidOutput, -255, 255);
  
  // Update last error
  lastError = error;
}

void applyLineFollowing() {
  if (!lineFollowing) return;
  
  // Calculate motor speeds based on PID output
  int leftSpeed = baseSpeed + pidOutput;
  int rightSpeed = baseSpeed - pidOutput;
  
  // Constrain speeds
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // Apply to DC motors for steering
  leftMotor(leftSpeed);
  rightMotor(rightSpeed);
  
  // Enhanced ESC thrust control based on line position and PID output
  float leftThrust = baseThrottle;
  float rightThrust = baseThrottle;
  
  // Position-based thrust adjustment (more responsive to line position)
  float positionFactor = abs(linePosition) / 30.0; // Normalize to 0-1 based on max position
  
  // Dynamic thrust multiplier based on position and PID
  float thrustMultiplier = 1.0 + (positionFactor * 0.4); // Up to 40% increase for extreme positions
  
  // Apply base thrust with position-based scaling
  leftThrust = baseThrottle * thrustMultiplier;
  rightThrust = baseThrottle * thrustMultiplier;
  
  // Enhanced differential thrust for turning assistance
  if (linePosition > 0) { // Line is to the right, need to turn right
    // Increase left ESC thrust for right turn
    leftThrust += (positionFactor * 0.3);
    // Decrease right ESC thrust for right turn
    rightThrust -= (positionFactor * 0.2);
  } else if (linePosition < 0) { // Line is to the left, need to turn left
    // Increase right ESC thrust for left turn
    rightThrust += (positionFactor * 0.3);
    // Decrease left ESC thrust for left turn
    leftThrust -= (positionFactor * 0.2);
  }
  
  // Additional PID-based thrust adjustment for quick corrections
  float pidFactor = abs(pidOutput) / baseSpeed; // Normalize PID output
  if (pidFactor > 0.1) { // Only apply if PID output is significant
    // Add thrust boost for quick corrections
    float correctionBoost = pidFactor * 0.25; // Up to 25% boost for corrections
    
    if (pidOutput > 0) { // Right correction needed
      leftThrust += correctionBoost;
      rightThrust += correctionBoost * 0.5; // Less boost to right
    } else { // Left correction needed
      rightThrust += correctionBoost;
      leftThrust += correctionBoost * 0.5; // Less boost to left
    }
  }
  
  // Constrain ESC values
  leftThrust = constrain(leftThrust, 0.0, 1.0);
  rightThrust = constrain(rightThrust, 0.0, 1.0);
  
  // Apply ESC inversions
  if (esc1Inverted) {
    // For inverted ESCs, we need to handle this differently
    // This would require more complex logic depending on ESC type
  }
  if (esc2Inverted) {
    // For inverted ESCs, we need to handle this differently  
    // This would require more complex logic depending on ESC type
  }
  
  // Update ESC motor values with swapping if configured
  if (escSwapped) {
    motorval[0] = rightThrust;  // ESC1 gets right thrust
    motorval[1] = leftThrust;   // ESC2 gets left thrust
  } else {
    motorval[0] = leftThrust;   // ESC1 gets left thrust
    motorval[1] = rightThrust;  // ESC2 gets right thrust
  }
  
  // Send ESC commands
  dshot.write(motorval);
  
  // Debug: Print PID status if enabled
  if (pidStatusEnabled) {
    printPIDStatus(leftSpeed, rightSpeed, leftThrust, rightThrust, positionFactor, pidFactor);
  }
}

extern "C" void DMA2_Stream1_IRQHandler(void) {
  dshot.handleDmaIrqStream1();
}
extern "C" void DMA2_Stream2_IRQHandler(void) {
  dshot.handleDmaIrqStream2();
}

void checkSerial() {
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim();

    Serial1.print("Got command: '");
    Serial1.print(cmd);
    Serial1.println("'");

    if (cmd.equalsIgnoreCase("start")) {
      sweeping = true;
      lineFollowing = false;
      Serial1.println("Sweep STARTED");
    } else if (cmd.equalsIgnoreCase("stop")) {
      sweeping = false;
      lineFollowing = false;
      motorval[0] = 0.0;
      motorval[1] = 0.0;
      leftMotor(0);
      rightMotor(0);
      Serial1.println("Sweep STOPPED, ESC motors OFF");
    } else if (cmd.equalsIgnoreCase("linefollow")) {
      lineFollowing = true;
      sweeping = false;
      Serial1.println("Line following STARTED");
    } else if (cmd.equalsIgnoreCase("stopline")) {
      lineFollowing = false;
      leftMotor(0);
      rightMotor(0);
      Serial1.println("Line following STOPPED");
    } else if (cmd.equalsIgnoreCase("pid")) {
      Serial1.print("Kp: ");
      Serial1.print(Kp);
      Serial1.print(" Ki: ");
      Serial1.print(Ki);
      Serial1.print(" Kd: ");
      Serial1.print(Kd);
      Serial1.print(" Base Speed: ");
      Serial1.println(baseSpeed);
    }
    // ===== PID TUNING =====
    else if (cmd.startsWith("Kp=")) {
      float val = cmd.substring(3).toFloat();
      Kp = val;
      Serial1.print("Kp set to: ");
      Serial1.println(val);
    } else if (cmd.startsWith("Ki=")) {
      float val = cmd.substring(3).toFloat();
      Ki = val;
      Serial1.print("Ki set to: ");
      Serial1.println(val);
    } else if (cmd.startsWith("Kd=")) {
      float val = cmd.substring(3).toFloat();
      Kd = val;
      Serial1.print("Kd set to: ");
      Serial1.println(val);
    } else if (cmd.startsWith("basespeed=")) {
      int val = cmd.substring(10).toInt();
      baseSpeed = constrain(val, 0, 255);
      Serial1.print("Base speed set to: ");
      Serial1.println(baseSpeed);
    } else if (cmd.startsWith("thrust=")) {
      float val = cmd.substring(7).toFloat();
      baseThrottle = constrain(val, 0.0, 1.0);
      Serial1.print("Base ESC thrust set to: ");
      Serial1.println(baseThrottle, 2);
    }
    // ===== ESC INDIVIDUAL CONTROL =====
    else if (cmd.startsWith("M1=")) {
      float val = cmd.substring(3).toFloat();
      if (val >= 0.0 && val <= 1.0) {
        sweeping = false;
        lineFollowing = false;
        motorval[0] = val;
        Serial1.print("ESC Motor 1 set to: ");
        Serial1.println(val, 2);
      }
    } else if (cmd.startsWith("M2=")) {
      float val = cmd.substring(3).toFloat();
      if (val >= 0.0 && val <= 1.0) {
        sweeping = false;
        lineFollowing = false;
        motorval[1] = val;
        Serial1.print("ESC Motor 2 set to: ");
        Serial1.println(val, 2);
      }
    } else if (cmd.startsWith("both=")) {
      float val = cmd.substring(5).toFloat();
      if (val >= 0.0 && val <= 1.0) {
        sweeping = false;
        lineFollowing = false;
        motorval[0] = val;
        motorval[1] = val;
        Serial1.print("Both ESC motors set to: ");
        Serial1.println(val, 2);
      }
    }
    // ===== DC MOTOR CONTROL =====
    else if (cmd.startsWith("LM=")) {
      int val = cmd.substring(3).toInt();
      val = constrain(val, -255, 255);
      leftMotor(val);
      Serial1.print("Left DC motor set to: ");
      Serial1.println(val);
    } else if (cmd.startsWith("RM=")) {
      int val = cmd.substring(3).toInt();
      val = constrain(val, -255, 255);
      rightMotor(val);
      Serial1.print("Right DC motor set to: ");
      Serial1.println(val);
    } else if (cmd.startsWith("bothdc=")) {
      int val = cmd.substring(7).toInt();
      val = constrain(val, -255, 255);
      leftMotor(val);
      rightMotor(val);
      Serial1.print("Both DC motors set to: ");
      Serial1.println(val);
    } else if (cmd.equalsIgnoreCase("stopdc")) {
      leftMotor(0);
      rightMotor(0);
      Serial1.println("Both DC motors STOPPED");
    }
    // ===== DEBUG COMMANDS =====
    else if (cmd.equalsIgnoreCase("sensorprint")) {
      sensorPrintEnabled = !sensorPrintEnabled;
      Serial1.print("Sensor debug printing: ");
      Serial1.println(sensorPrintEnabled ? "ON" : "OFF");
    } else if (cmd.equalsIgnoreCase("pidstatus")) {
      pidStatusEnabled = !pidStatusEnabled;
      Serial1.print("PID status printing: ");
      Serial1.println(pidStatusEnabled ? "ON" : "OFF");
    }
    // ===== SETUP COMMANDS =====
    else if (cmd.startsWith("setup_linecolor=")) {
      String value = cmd.substring(16);
      followBlackLine = (value == "black");
      Serial1.print("Line color set to: ");
      Serial1.println(followBlackLine ? "black" : "white");
    } else if (cmd.startsWith("setup_leftmotor=")) {
      String value = cmd.substring(16);
      leftMotorInverted = (value == "inverted");
      Serial1.print("Left motor set to: ");
      Serial1.println(leftMotorInverted ? "inverted" : "normal");
    } else if (cmd.startsWith("setup_rightmotor=")) {
      String value = cmd.substring(17);
      rightMotorInverted = (value == "inverted");
      Serial1.print("Right motor set to: ");
      Serial1.println(rightMotorInverted ? "inverted" : "normal");
    } else if (cmd.startsWith("setup_escmapping=")) {
      String value = cmd.substring(17);
      escSwapped = (value == "swapped");
      Serial1.print("ESC mapping set to: ");
      Serial1.println(escSwapped ? "swapped" : "normal");
    } else if (cmd.startsWith("setup_esc1=")) {
      String value = cmd.substring(11);
      esc1Inverted = (value == "inverted");
      Serial1.print("ESC1 set to: ");
      Serial1.println(esc1Inverted ? "inverted" : "normal");
    } else if (cmd.startsWith("setup_esc2=")) {
      String value = cmd.substring(11);
      esc2Inverted = (value == "inverted");
      Serial1.print("ESC2 set to: ");
      Serial1.println(esc2Inverted ? "inverted" : "normal");
    } else {
      Serial1.println("Invalid input. Use: start/stop/linefollow/stopline/pid/Kp=val/Ki=val/Kd=val/basespeed=val/thrust=val/setup_*/M1=val/M2=val/both=val/LM=n/RM=n/bothdc=n/stopdc/sensorprint/pidstatus");
    }
  }
}

static void run(const uint32_t usec) {
  static uint32_t prev;

  if (usec - prev > 1000000 / UPDATE_RATE) {
    prev = usec;

    if (sweeping) {
      // Sweep both motors together
      if (increasing) {
        motorval[0] += STEP;
        motorval[1] += STEP;
        if (motorval[0] >= 1.0) increasing = false;
      } else {
        motorval[0] -= STEP;
        motorval[1] -= STEP;
        if (motorval[0] <= 0.0) increasing = true;
      }

      Serial1.print("Sweep throttle: ");
      Serial1.println(motorval[0], 2);
    } else if (lineFollowing) {
      // Line following mode - sensors read in separate timer
      // PID and motor control handled in separate timer
    } else {
      
    }

    // Send both motors (only if not line following)
    if (!lineFollowing) {
      dshot.write(motorval);
    }
  }
}

void setup(void) {
  analogReadResolution(12);
  Serial1.begin(115200);
  Serial1.println("DShot test ready with line following.");
  Serial1.println("Commands: 'start', 'stop', 'linefollow', 'stopline', 'pid', 'Kp=val', 'Ki=val', 'Kd=val', 'basespeed=val', 'thrust=val'");
  Serial1.println("Line following: 'linefollow' to start, 'stopline' to stop");
  Serial1.println("Debug: 'sensorprint' to toggle sensor values, 'pidstatus' to toggle PID debug");
  
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
  dshot.begin(pins);
  
  // Initialize timers
  lastPIDTime = millis();
}

void loop(void) {
  const auto usec = micros();
  unsigned long currentTime = millis();
  
  checkSerial();
  run(usec);
  
  // Combined sensor reading and PID control (50Hz)
  if (currentTime - lastPIDTime >= CONTROL_INTERVAL) {
    lastPIDTime = currentTime;
    if (lineFollowing) {
      readSensors();
      updatePID();
      applyLineFollowing();
    }
  }
}
