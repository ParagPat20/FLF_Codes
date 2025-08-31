// Arduino Line Following Robot - Linear Array + Classic PID
// - Linear 7-sensor centroid (-3..+3), error = pos - 0
// - Simple PID loop with normalized error scaling
// - Basic recovery/search using last turn direction memory
// - Automatic sensor calibration on boot

#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial Serial1(USART1);

// ===== Linear Sensor Array =====
// 7-sensor array: L3, L2, L1, M0, R1, R2, R3
const uint8_t sensorPins[7] = { PA4, PA5, PA6, PA0, PA1, PA2, PA3 };

// Dynamic thresholds (lower reading = black line)
int sensorThresholds[7] = { 2450, 2700, 3100, 3100, 3100, 3100, 2500 };

// Normalized position mapping for centroid (-3..+3) - FIXED OSCILLATION ISSUE
const int SENSOR_POS[7] = { -3, -2, -1, 0, 1, 2, 3 };
static const int CENTER_POS = 0;

// ===== DC Motor Driver Pins =====
#define PWMA PA15
#define AIN2 PB14
#define AIN1 PB15
#define BIN1 PA8
#define BIN2 PB3
#define PWMB PB4

// LED for status
#define LED_PIN PC13

// ===== Timers =====
#define SAMPLE_INTERVAL 1  // ms - INCREASED for stability
#define PID_INTERVAL 1     // ms - INCREASED for stability

// ===== CALIBRATION VARIABLES =====
#define CALIBRATION_DELAY 20     // Delay between samples during rotation
#define CALIBRATION_ROTATION_SPEED 180  // Rotation speed for smooth movement
bool calibrationComplete = false;
int sensorMinValues[7] = {4095, 4095, 4095, 4095, 4095, 4095, 4095};  // Start with max ADC value
int sensorMaxValues[7] = {0, 0, 0, 0, 0, 0, 0};                        // Start with min ADC value
int calibratedThresholds[7] = {0, 0, 0, 0, 0, 0, 0};                   // Will hold calibrated thresholds

// ===== PID - TUNED FOR NORMALIZED ERROR SCALE =====
float Kp = 3.0f;   // INCREASED for normalized scale (-3 to +3)
float Ki = 0.15f;  // START WITH 0 to avoid integral windup
float Kd = 0.2f;   // START WITH 0, add after P is stable

int baseSpeed = 160;
int currentSpeed = 160;

int MAX_CORRECTION = 400;
int ERROR_DEADBAND = 0;

float error = 0, lastError = 0, integral = 0;
unsigned long lastPidTime = 0;
unsigned long lastSampleTime = 0;

// ===== Line/State =====
bool lineDetected = false;
unsigned long lineDetectedTime = 0;
float currentPosition = CENTER_POS;
int lastTurnDirection = 0;  // -1 left, +1 right, 0 straight
bool inRecoveryMode = false;
unsigned long recoveryStartTime = 0;

bool robotRunning = true;
unsigned long bootTime = 0;

// ===== Forward Decls =====
static void executeLineFollowing();
static void run();
static void performSensorCalibration();
static void displayCalibrationStatus();

// ===== Normalized linear position reading: -3..+3, or -999 if none =====
float readLinePositionLinear() {
  long weightedSum = 0;
  long weight = 0;
  int activeCount = 0;

  for (int i = 0; i < 7; i++) {
    int v = analogRead(sensorPins[i]);
    // black line detection: reading below threshold
    if (v < sensorThresholds[i]) {
      weightedSum += (long)SENSOR_POS[i];
      weight += 1;
      activeCount++;
    }
  }

  if (activeCount == 0 || weight == 0) return -999.0f;  // no line
  float pos = (float)weightedSum / (float)weight;
  // clamp to bounds
  if (pos < -3.0f) pos = -3.0f;
  if (pos > 3.0f) pos = 3.0f;
  return pos;
}

// ===== Motor helpers =====
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

void stopMotors() {
  leftMotor(0);
  rightMotor(0);
}

// ===== Recovery patterns =====
void sharpLeftTurn() {
  leftMotor(-200);
  rightMotor(200);
}
void sharpRightTurn() {
  leftMotor(200);
  rightMotor(-200);
}

void fastLeftSearch() {
  leftMotor(50);
  rightMotor(200);
}
void fastRightSearch() {
  leftMotor(200);
  rightMotor(50);
}

// ===== Main LF logic (normalized + stable PID) =====
void executeLineFollowing() {
  if (!robotRunning) {
    stopMotors();
    return;
  }

  unsigned long now = millis();


  float pos = readLinePositionLinear();
  if (pos > -900.0f) {  // Line detected (not -999)
    lineDetected = true;
    lineDetectedTime = now;
    currentPosition = pos;

    // update last turn bias from side dominance
    if (pos < CENTER_POS - 0.5f) lastTurnDirection = -1;
    else if (pos > CENTER_POS + 0.5f) lastTurnDirection = +1;
    else lastTurnDirection = 0;

    inRecoveryMode = false;
  } else {
    lineDetected = false;
  }

  if (lineDetected && (now - lineDetectedTime) < 100) {
    // Normal PID on normalized lateral error
    error = currentPosition - CENTER_POS;  // Now error is -3 to +3

    // Deadband & anti-wobble
    if (abs(error) < ERROR_DEADBAND) {
      error = 0;
      integral = 0;
    }

    float dt = PID_INTERVAL / 1000.0f;

    // Integral control (currently disabled to avoid windup)
    if (Ki > 0) {
      integral += error * dt;
    }

    // Derivative control (currently disabled to avoid spikes)
    float derivative = 0;
    if (Kd > 0) {
      derivative = (error - lastError) / dt;
    }

    float rawCorrection = Kp * error + Ki * integral + Kd * derivative;

    // clamp correction to prevent oscillation
    float correction = constrain(rawCorrection, -MAX_CORRECTION, MAX_CORRECTION);
    lastError = error;

    // Motor speeds - keep both motors mostly forward
    int leftSpeed = currentSpeed + (int)correction;
    int rightSpeed = currentSpeed - (int)correction;



    leftMotor(constrain(leftSpeed, -255, 255));
    rightMotor(constrain(rightSpeed, -255, 255));
  } else {
    // === Recovery/Search when line lost ===
    unsigned long timeSinceLoss = now - lineDetectedTime;

    // Start recovery mode only after 100ms to avoid false triggers
    if (timeSinceLoss >= 100) {
      if (!inRecoveryMode) {
        inRecoveryMode = true;
        recoveryStartTime = now;
      }

      if (timeSinceLoss < 2000) {
        // Use last turn memory to bias
        if (lastTurnDirection == -1) {
          if (timeSinceLoss < 200) {
            // Small corrective left nudge
            leftMotor(-50);
            rightMotor(120);
          } else if (timeSinceLoss < 600) {
            // If still lost, then sharper left
            sharpLeftTurn();
          } else {
            // Fallback fast search
            fastLeftSearch();
          }
        } else if (lastTurnDirection == +1) {
          if (timeSinceLoss < 200) {
            // Small corrective right nudge
            leftMotor(120);
            rightMotor(-50);
          } else if (timeSinceLoss < 600) {
            // If still lost, then sharper right
            sharpRightTurn();
          } else {
            // Fallback fast search
            fastRightSearch();
          }
        } else {
          // No bias → try left then right then sweep
          if (timeSinceLoss < 300) {
            sharpLeftTurn();
          } else if (timeSinceLoss < 600) {
            sharpRightTurn();
          } else {
            fastLeftSearch();
          }
        }
      } else {
        // Timeout → stop & reset
        stopMotors();
        integral = 0;
        currentPosition = CENTER_POS;
      }
    }
  }
}

// ===== CALIBRATION FUNCTIONS =====
void performSensorCalibration() {
  Serial1.println("=== STARTING SENSOR CALIBRATION ===");
  Serial1.println("Place robot on white surface (no black line)");
  Serial1.println("Robot will rotate clockwise for 3 seconds during calibration");
  Serial1.println("Calibrating in 3 seconds...");
  
  // Turn LED ON during calibration
  digitalWrite(LED_PIN, HIGH);
  
  // Wait for user to place robot on white surface
  delay(1000);
  
  Serial1.println("Starting clockwise rotation...");
  
  // Start clockwise rotation and keep it going for 3 seconds
  leftMotor(CALIBRATION_ROTATION_SPEED);
  rightMotor(-CALIBRATION_ROTATION_SPEED);
  
  // Take samples while rotating for exactly 3 seconds
  unsigned long startTime = millis();
  unsigned long sampleCount = 0;
  
  while (millis() - startTime < 3000) {  // Rotate for exactly 3 seconds
    // Read all sensors
    for (int sensor = 0; sensor < 7; sensor++) {
      int value = analogRead(sensorPins[sensor]);
      
      // Update min and max values for each sensor
      if (value < sensorMinValues[sensor]) {
        sensorMinValues[sensor] = value;
      }
      if (value > sensorMaxValues[sensor]) {
        sensorMaxValues[sensor] = value;
      }
    }
    
    sampleCount++;
    
    // Keep rotating - no direction changes, just continuous clockwise rotation
    delay(CALIBRATION_DELAY);
  }
  
  // Stop motors after calibration
  stopMotors();
  Serial1.println("Calibration rotation complete, stopping motors...");
  
  // Calculate calibrated thresholds (middle value between min and max)
  Serial1.println("Calculating calibrated thresholds...");
  for (int sensor = 0; sensor < 7; sensor++) {
    // Calculate threshold as middle value between min and max
    calibratedThresholds[sensor] = (sensorMinValues[sensor] + sensorMaxValues[sensor]) / 2;
    
    // Add a small offset to make detection more reliable
    // This ensures the threshold is slightly below the middle for better black line detection
    int offset = (sensorMaxValues[sensor] - sensorMinValues[sensor]) * 0.2;  // 20% offset
    calibratedThresholds[sensor] -= offset;
    
    // Ensure threshold is within valid range
    calibratedThresholds[sensor] = constrain(calibratedThresholds[sensor], 0, 4095);
    
    // Print calibration results for each sensor
    Serial1.print("Sensor ");
    Serial1.print(sensor);
    Serial1.print(": Min=");
    Serial1.print(sensorMinValues[sensor]);
    Serial1.print(", Max=");
    Serial1.print(sensorMaxValues[sensor]);
    Serial1.print(", Threshold=");
    Serial1.println(calibratedThresholds[sensor]);
  }
  
  // Update the global sensor thresholds with calibrated values
  for (int i = 0; i < 7; i++) {
    sensorThresholds[i] = calibratedThresholds[i];
  }
  
  calibrationComplete = true;
  Serial1.println("=== CALIBRATION COMPLETE ===");
  Serial1.println("Place robot on black line to test...");
  Serial1.println("Waiting 3 seconds before starting line following...");
  
  // Turn LED OFF after calibration is complete
  digitalWrite(LED_PIN, LOW);
  
  delay(1000);
}

void displayCalibrationStatus() {
  if (calibrationComplete) {
    Serial1.println("=== CALIBRATION STATUS ===");
    for (int i = 0; i < 7; i++) {
      Serial1.print("Sensor ");
      Serial1.print(i);
      Serial1.print(": Threshold=");
      Serial1.print(sensorThresholds[i]);
      Serial1.print(" (Min=");
      Serial1.print(sensorMinValues[i]);
      Serial1.print(", Max=");
      Serial1.print(sensorMaxValues[i]);
      Serial1.println(")");
    }
    Serial1.println("========================");
  } else {
    Serial1.println("Calibration not yet performed!");
  }
}

// ===== Setup / Loop =====
void setup() {
  analogReadResolution(12);
  Serial1.begin(115200);
  for (int i = 0; i < 7; i++) pinMode(sensorPins[i], INPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Wait for serial to be ready
  delay(1000);
  
  Serial1.println("=== ARDUINO LINE FOLLOWER ROBOT ===");
  Serial1.println("Starting sensor calibration...");
  
  // Perform sensor calibration on boot
  performSensorCalibration();
  
  // Display calibration results
  displayCalibrationStatus();
  
  Serial1.println("=== ROBOT READY FOR LINE FOLLOWING ===");
  Serial1.println("Place robot on black line to start...");

  bootTime = millis();
}

void loop() {
  if (millis() - bootTime > 3000) {
    run();
  }
}

static void run() {
  executeLineFollowing();
}


