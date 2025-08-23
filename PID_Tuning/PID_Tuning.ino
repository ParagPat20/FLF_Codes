// PID Tuning Robot - Pure PID Control for Parameter Optimization
// 20kHz PID loop with serial parameter adjustment
// Robot tilts in place to stay on line (no forward movement)
// Raw PID calculations with no filters or limits

#include <Arduino.h>
#include <HardwareSerial.h>

// HardwareSerial instance on PA9 (TX) / PA10 (RX)
HardwareSerial Serial1(USART1);

// ===== PIN DEFINITIONS =====
// 7-sensor array: L3, L2, L1, M0(center), R1, R2, R3
const uint8_t sensorPins[7] = { PA4, PA5, PA6, PA0, PA1, PA2, PA3 };
// Fixed threshold values based on testing - values below threshold indicate black line
const int sensorThresholds[7] = { 2550, 2880, 3000, 3500, 3150, 3125, 2600 };

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
#define SAMPLE_INTERVAL 1  // 1ms sensor reading
#define PID_INTERVAL 50    // 50µs = 20kHz PID loop

// ===== RAW PID VARIABLES =====
// Pure PID gains - adjustable via serial
float Kp = 1.0;   // Proportional gain
float Ki = 0.1;   // Integral gain
float Kd = 0.05;  // Derivative gain

// PID calculation variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float pidOutput = 0;

// Timing variables
unsigned long lastPidTime = 0;
unsigned long lastSampleTime = 0;
unsigned long lastSerialTime = 0;

// Line detection variables
bool lineDetected = false;
float currentPosition = 3000;  // Center position (sensor 3)

// Motor control parameters
int tiltSpeed = 100;  // Base tilt speed for staying on line
int throttle = 0;     // Forward/backward speed (-255 to +255)

// ===== INITIALIZATION FUNCTION =====
void initializePIDTuning() {
  Serial1.begin(115200);
  Serial1.println("=== PID TUNING MODE - READY ===");
  Serial1.println("Fixed thresholds loaded, no calibration needed.");

  digitalWrite(LED_PIN, HIGH);  // Turn LED on to indicate ready

  Serial1.println("\nPID TUNING COMMANDS:");
  Serial1.println("P<value> - Set Kp (e.g., P1.5)");
  Serial1.println("I<value> - Set Ki (e.g., I0.2)");
  Serial1.println("D<value> - Set Kd (e.g., D0.05)");
  Serial1.println("S<value> - Set tilt speed (e.g., S120)");
  Serial1.println("T<value> - Set throttle -255 to +255 (e.g., T150)");
  Serial1.println("R - Reset PID values to default");
  Serial1.println("H - Show this help");
  Serial1.print("\nCurrent PID: Kp=");
  Serial1.print(Kp);
  Serial1.print(", Ki=");
  Serial1.print(Ki);
  Serial1.print(", Kd=");
  Serial1.println(Kd);

  delay(1000);
}

// ===== SENSOR READING =====
float readLinePosition() {
  int sumIndex = 0;
  int blackCount = 0;

  // Read all 7 sensors once
  for (int i = 0; i < 7; i++) {
    int value = analogRead(sensorPins[i]);

    // Use fixed threshold for each sensor
    // Lower values indicate black line (we follow black)
    if (value < sensorThresholds[i]) {  // Black line detected
      sumIndex += i;
      blackCount++;
    }
  }

  // Return position if we detected black line, otherwise return -1 to indicate no line
  if (blackCount > 0) {
    float positionIndex = (float)sumIndex / blackCount;
    // Convert to position value: 0-6000 range where 3000 is exact center (sensor 3, PA0)
    return positionIndex * 1000;
  } else {
    return -1;  // No line detected
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

void stopMotors() {
  leftMotor(0);
  rightMotor(0);
}

// ===== SERIAL COMMAND PROCESSING =====
void processSerialCommands() {
  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      char cmd = command.charAt(0);
      float value = command.substring(1).toFloat();

      switch (cmd) {
        case 'P':
        case 'p':
          Kp = value;
          Serial1.print("Kp set to: ");
          Serial1.println(Kp);
          break;

        case 'I':
        case 'i':
          Ki = value;
          Serial1.print("Ki set to: ");
          Serial1.println(Ki);
          break;

        case 'D':
        case 'd':
          Kd = value;
          Serial1.print("Kd set to: ");
          Serial1.println(Kd);
          break;

        case 'S':
        case 's':
          tiltSpeed = (int)value;
          Serial1.print("Tilt speed set to: ");
          Serial1.println(tiltSpeed);
          break;

        case 'T':
        case 't':
          throttle = constrain((int)value, -255, 255);
          Serial1.print("Throttle set to: ");
          Serial1.println(throttle);
          break;

        case 'R':
        case 'r':
          Kp = 1.0;
          Ki = 0.1;
          Kd = 0.05;
          tiltSpeed = 100;
          throttle = 0;
          Serial1.println("PID values reset to default");
          break;

        case 'H':
        case 'h':
          Serial1.println("\nPID TUNING COMMANDS:");
          Serial1.println("P<value> - Set Kp");
          Serial1.println("I<value> - Set Ki");
          Serial1.println("D<value> - Set Kd");
          Serial1.println("S<value> - Set tilt speed");
          Serial1.println("T<value> - Set throttle (-255 to +255)");
          Serial1.println("R - Reset to default");
          Serial1.println("H - Show help");
          break;

        default:
          Serial1.println("Unknown command. Type H for help.");
          break;
      }

      // Show current values after any change
      if (cmd != 'H' && cmd != 'h') {
        Serial1.print("Current: Kp=");
        Serial1.print(Kp);
        Serial1.print(", Ki=");
        Serial1.print(Ki);
        Serial1.print(", Kd=");
        Serial1.print(Kd);
        Serial1.print(", Speed=");
        Serial1.print(tiltSpeed);
        Serial1.print(", Throttle=");
        Serial1.println(throttle);
      }
    }
  }
}

// ===== SETUP =====
void setup() {
  analogReadResolution(12);

  // Initialize sensor pins
  for (int i = 0; i < 7; i++) pinMode(sensorPins[i], INPUT);

  // Initialize motor pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize PID tuning mode with fixed thresholds
  initializePIDTuning();
}

// ===== MAIN LOOP =====
void loop() {
  unsigned long currentTime = micros();  // Use microseconds for 20kHz timing

  // Read sensors at 1kHz
  if (currentTime - lastSampleTime >= 1000) {  // 1000µs = 1ms
    lastSampleTime = currentTime;

    float rawPosition = readLinePosition();

    if (rawPosition >= 0) {
      currentPosition = rawPosition;
      lineDetected = true;
    } else {
      lineDetected = false;
    }
  }

  // RAW PID calculation at 20kHz
  if (currentTime - lastPidTime >= PID_INTERVAL) {  // 50µs = 20kHz
    lastPidTime = currentTime;

    if (lineDetected) {
      // Calculate error (center is 3000)
      error = 3000.0 - currentPosition;

      // RAW PID calculations - NO FILTERS OR LIMITS
      float deltaTime = PID_INTERVAL / 1000000.0;  // Convert µs to seconds

      integral += error * deltaTime;
      derivative = (error - lastError) / deltaTime;

      // Pure PID output
      pidOutput = Kp * error + Ki * integral + Kd * derivative;

      lastError = error;

      // Apply to motors: throttle + PID correction
      int leftSpeed = throttle - pidOutput;   // Base speed + left correction
      int rightSpeed = throttle + pidOutput;  // Base speed + right correction

      // Constrain to motor limits
      leftSpeed = constrain(leftSpeed, -255, 255);
      rightSpeed = constrain(rightSpeed, -255, 255);

      leftMotor(leftSpeed);
      rightMotor(rightSpeed);

    } else {
      // No line detected - stop motors
      stopMotors();
      integral = 0;  // Reset integral when line lost
    }
  }

  // Process serial commands at 50Hz
  if (currentTime - lastSerialTime >= 2000) {  // 20000µs = 20ms = 50Hz
    lastSerialTime = currentTime;
    processSerialCommands();

    // Print status every 50ms for faster graphing
    static unsigned long lastStatusTime = 0;
    if (currentTime - lastStatusTime >= 5000) {  // 50ms = 20Hz
      lastStatusTime = currentTime;

      // Format for Arduino Serial Plotter - comma separated values with PID parameters
      Serial1.print("Setpoint:");
      Serial1.print(0);  // Target error (always 0 for perfect line following)
      Serial1.print(",Error:");
      Serial1.print(error);
      Serial1.print(",Position:");
      Serial1.print(currentPosition);
      Serial1.print(",Target_Pos:");
      Serial1.print(3000);  // Target position (center)
      Serial1.print(",PID_Output:");
      Serial1.print(pidOutput);
      Serial1.print(",Integral:");
      Serial1.print(integral);
      Serial1.print(",Derivative:");
      Serial1.print(derivative);
      Serial1.print(",Kp:");
      Serial1.print(Kp);
      Serial1.print(",Ki:");
      Serial1.print(Ki);
      Serial1.print(",Kd:");
      Serial1.print(Kd);
      Serial1.print(",Speed:");
      Serial1.print(tiltSpeed);
      Serial1.print(",Throttle:");
      Serial1.println(throttle);
    }
  }
}
