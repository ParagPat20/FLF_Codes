// Arduino Line Following Robot - Linear Array + Classic PID
// - Linear 7-sensor centroid (-3..+3), error = pos - 0
// - Simple PID loop with normalized error scaling
// - Adaptive thrust mixing for BLDC thrusters via DShot
// - Basic recovery/search using last turn direction memory

#include <Arduino.h>
#include <dshot_stm32f4.h>
#include <HardwareSerial.h>
#include <vector>

HardwareSerial Serial1(USART1);

// ===== ESC (Thrusters) =====
static const uint8_t PIN1 = PB8;  // Left Thruster
static const uint8_t PIN2 = PB9;  // Right Thruster
static std::vector<uint8_t> pins = { PIN1, PIN2 };
static Stm32F4Dshot dshot;
static float thrusterValues[2] = { 0.0f, 0.0f };

// ===== DMA IRQ for DShot (as per your setup) =====
extern "C" void DMA2_Stream1_IRQHandler(void) {
  dshot.handleDmaIrqStream1();
}
extern "C" void DMA2_Stream2_IRQHandler(void) {
  dshot.handleDmaIrqStream2();
}

// ===== Linear Sensor Array =====
// 7-sensor array: L3, L2, L1, M0, R1, R2, R3
const uint8_t sensorPins[7] = { PA4, PA5, PA6, PA0, PA1, PA2, PA3 };

// Dynamic thresholds (lower reading = black line)
int sensorThresholds[7] = { 2500, 2700, 2850, 3200, 2900, 2800, 2400 };

// Normalized position mapping for centroid (-3..+3) - FIXED OSCILLATION ISSUE
const int SENSOR_POS[7] = { -3, -2, -1, 0, 1, 2, 3 };
static const int CENTER_POS = 0;

// ===== DC Motor Driver Pins =====
#define PWMA PB7
#define AIN2 PB14
#define AIN1 PB15
#define BIN1 PA8
#define BIN2 PB3
#define PWMB PB4

// LED for status
#define LED_PIN PC13

// ===== Timers =====
#define SAMPLE_INTERVAL 5  // ms - INCREASED for stability
#define PID_INTERVAL 5     // ms - INCREASED for stability

// ===== Thrust control (for BLDC thrusters) =====
#define MIN_THRUST 0.2f
#define MAX_THRUST 0.9f
#define BASE_THRUST 0.5f
#define THRUST_ADJUSTMENT_FACTOR 0.6f  // how strongly correction changes thrusts

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
static void runESCTiming(const uint32_t usec);
static void setThrustValues(float leftThrust, float rightThrust);
static void disableThrusters();
static void executeLineFollowing();
static void run();

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

  // Sample sensors
  if (now - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = now;

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
  }

  // PID loop
  if (now - lastPidTime >= PID_INTERVAL) {
    lastPidTime = now;

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

      // Thrust mixing (BLDC via DShot) - REDUCED for smoother operation
      float thrustAdj = (correction / 100.0f) * THRUST_ADJUSTMENT_FACTOR;  // REDUCED divisor
      float leftThrust = constrain(BASE_THRUST + thrustAdj, MIN_THRUST, MAX_THRUST);
      float rightThrust = constrain(BASE_THRUST - thrustAdj, MIN_THRUST, MAX_THRUST);

      setThrustValues(leftThrust, rightThrust);

      leftMotor(constrain(leftSpeed, -255, 255));
      rightMotor(constrain(rightSpeed, -255, 255));
    } else {
      // === Recovery/Search when line lost ===
      if (!inRecoveryMode) {
        inRecoveryMode = true;
        recoveryStartTime = now;
      }

      unsigned long timeSinceLoss = now - lineDetectedTime;

      if (timeSinceLoss < 2000) {
        // Use last turn memory to bias
        if (lastTurnDirection == -1) {
          if (timeSinceLoss < 200) {
            // Small corrective left nudge
            leftMotor(-50);
            rightMotor(120);
            setThrustValues(MIN_THRUST * 0.8f, MAX_THRUST * 0.8f);
          } else if (timeSinceLoss < 600) {
            // If still lost, then sharper left
            sharpLeftTurn();
            setThrustValues(MIN_THRUST, MAX_THRUST);
          } else {
            // Fallback fast search
            fastLeftSearch();
            setThrustValues(MIN_THRUST * 1.6f, MAX_THRUST * 0.8f);
          }
        } else if (lastTurnDirection == +1) {
          if (timeSinceLoss < 200) {
            // Small corrective right nudge
            leftMotor(120);
            rightMotor(-50);
            setThrustValues(MAX_THRUST * 0.8f, MIN_THRUST * 0.8f);
          } else if (timeSinceLoss < 600) {
            // If still lost, then sharper right
            sharpRightTurn();
            setThrustValues(MAX_THRUST, MIN_THRUST);
          } else {
            // Fallback fast search
            fastRightSearch();
            setThrustValues(MAX_THRUST * 0.8f, MIN_THRUST * 1.6f);
          }
        } else {
          // No bias → try left then right then sweep
          if (timeSinceLoss < 300) {
            sharpLeftTurn();
            setThrustValues(MIN_THRUST * 1.4f, MAX_THRUST * 0.8f);
          } else if (timeSinceLoss < 600) {
            sharpRightTurn();
            setThrustValues(MAX_THRUST * 0.8f, MIN_THRUST * 1.4f);
          } else {
            fastLeftSearch();
            setThrustValues(MIN_THRUST * 1.6f, MAX_THRUST * 0.8f);
          }
        }
      } else {
        // Timeout → stop & reset
        stopMotors();
        disableThrusters();
        integral = 0;
        currentPosition = CENTER_POS;
      }
  }
}

// ===== Setup / Loop =====
void setup() {
  analogReadResolution(12);

  for (int i = 0; i < 7; i++) pinMode(sensorPins[i], INPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  dshot.begin(pins);

  bootTime = millis();
}

void loop() {
  const auto usec = micros();
  runESCTiming(usec);

  if (millis() - bootTime > 3000) {
    run();
  }
}

static void run() {
  executeLineFollowing();
}

// ===== Thrust helpers =====
void setThrustValues(float leftThrust, float rightThrust) {
  thrusterValues[0] = constrain(leftThrust, MIN_THRUST, MAX_THRUST);
  thrusterValues[1] = constrain(rightThrust, MIN_THRUST, MAX_THRUST);
}

void disableThrusters() {
  thrusterValues[0] = 0.0f;
  thrusterValues[1] = 0.0f;
}

// ===== ESC timing (DShot keepalive) =====
static void runESCTiming(const uint32_t usec) {
  static uint32_t prev;
  const uint32_t UPDATE_RATE = 50;  // Hz

  if (usec - prev > 1000000 / UPDATE_RATE) {
    prev = usec;
    dshot.write(thrusterValues);
  }
}
