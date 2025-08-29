/*
   Combined Motor Driver (TB6612) and ESC (DShot) Test Program
   
   Motor Driver Test (Default Mode):
   - Single character commands for TB6612 motor driver testing
   
   ESC Test Mode:
   - String commands for DShot ESC control
*/

#include <HardwareSerial.h>
#include <dshot_stm32f4.h>
#include <vector>

// HardwareSerial instance on PA9 (TX) / PA10 (RX)
HardwareSerial Serial1(USART1);

// ============ Motor Driver Variables ============
// Motor pins for TB6612
const int motorA1 = PB15;  // Left motor
const int motorA2 = PB14;
const int motorPWMA = PB1;

const int motorB1 = PA8;  // Right motor
const int motorB2 = PB3;
const int motorPWMB = PB4;

const int testSpeed = 255;  // Test speed (0-255)
const int testDuration = 3000;  // How long to run each test (ms)

// ============ ESC/DShot Variables ============
static const uint8_t PIN1 = PB8;   // Motor 1
static const uint8_t PIN2 = PB9;   // Motor 2

static std::vector<uint8_t> pins = {PIN1, PIN2};
static Stm32F4Dshot dshot;

// Motor throttle values [0.0 – 1.0]
static float motorval[2] = {0.0, 0.0};  

// Modes
static bool sweeping = false;
static bool increasing = true;

// Sweep settings
static const uint32_t UPDATE_RATE = 50;  // Hz (20 ms)
static const float STEP = 0.01;          // sweep step

// Test mode selection
static bool escMode = false;  // false = motor driver test, true = ESC test

// DMA interrupt handlers for DShot
extern "C" void DMA2_Stream1_IRQHandler(void) { dshot.handleDmaIrqStream1(); }
extern "C" void DMA2_Stream2_IRQHandler(void) { dshot.handleDmaIrqStream2(); }

void setup() {
  Serial1.begin(115200);
  
  // Initialize motor driver pins
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorPWMA, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorPWMB, OUTPUT);
  
  // Initialize DShot for ESCs
  dshot.begin(pins);
  
  // Initially stop all motors
  stopMotorDrivers();
  
  Serial1.println("Motor Test Program (Motor Driver + ESC)");
  Serial1.println("Commands:");
  Serial1.println("m: Switch to Motor Driver Test Mode");
  Serial1.println("e: Switch to ESC/DShot Test Mode");
  Serial1.println("");
  Serial1.println("Motor Driver Mode:");
  Serial1.println("1: Test Left Motor Forward");
  Serial1.println("2: Test Left Motor Backward");
  Serial1.println("3: Test Right Motor Forward");
  Serial1.println("4: Test Right Motor Backward");
  Serial1.println("5: Test Both Motors Forward");
  Serial1.println("6: Test Both Motors Backward");
  Serial1.println("7: Test Spin Left");
  Serial1.println("8: Test Spin Right");
  Serial1.println("0: Stop Motors");
  Serial1.println("");
  Serial1.println("ESC Mode:");
  Serial1.println("start: Begin sweep (min->max->min loop)");
  Serial1.println("stop: Stop sweep & motors off");
  Serial1.println("M1=0.3: Set Motor 1 throttle (0.0-1.0)");
  Serial1.println("M2=0.7: Set Motor 2 throttle (0.0-1.0)");
  Serial1.println("both=0.5: Set both motors to same throttle");
  Serial1.println("");
  Serial1.println("Current Mode: Motor Driver");
}

void loop() {
  if (escMode) {
    // ESC mode - run exactly like standalone ESC code
    const auto usec = micros();
    checkSerial();
    run(usec);
  } else {
    // Motor driver mode - handle single character commands
    if (Serial1.available()) {
      char cmd = Serial1.read();
      
      switch (cmd) {
        case 'm':
          escMode = false;
          Serial1.println("Switched to Motor Driver Mode");
          stopMotorDrivers();
          break;
          
        case 'e':
          escMode = true;
          stopMotorDrivers();  // Stop motor drivers
          motorval[0] = 0.0;   // Reset ESC values
          motorval[1] = 0.0;
          sweeping = false;
          Serial1.println("Switched to ESC/DShot Mode");
          Serial1.println("DShot test ready.");
          Serial1.println("Commands: 'start', 'stop', 'M1=val', 'M2=val', 'both=val'");
          break;
          
        case '1':
          Serial1.println("Testing Left Motor Forward");
          testLeftMotor(true);
          break;
          
        case '2':
          Serial1.println("Testing Left Motor Backward");
          testLeftMotor(false);
          break;
          
        case '3':
          Serial1.println("Testing Right Motor Forward");
          testRightMotor(true);
          break;
          
        case '4':
          Serial1.println("Testing Right Motor Backward");
          testRightMotor(false);
          break;
          
        case '5':
          Serial1.println("Testing Both Motors Forward");
          testBothMotors(true);
          break;
          
        case '6':
          Serial1.println("Testing Both Motors Backward");
          testBothMotors(false);
          break;
          
        case '7':
          Serial1.println("Testing Spin Left");
          testSpin(true);
          break;
          
        case '8':
          Serial1.println("Testing Spin Right");
          testSpin(false);
          break;
          
        case '0':
          Serial1.println("Stopping Motors");
          stopMotorDrivers();
          break;
      }
    }
  }
}

// ============ Motor Driver Functions ============

void testLeftMotor(bool forward) {
  stopMotorDrivers();
  if (forward) {
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
  } else {
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
  }
  analogWrite(motorPWMA, testSpeed);
  delay(testDuration);
  stopMotorDrivers();
}

void testRightMotor(bool forward) {
  stopMotorDrivers();
  if (forward) {
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
  } else {
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
  }
  analogWrite(motorPWMB, testSpeed);
  delay(testDuration);
  stopMotorDrivers();
}

void testBothMotors(bool forward) {
  stopMotorDrivers();
  if (forward) {
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
  } else {
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
  }
  analogWrite(motorPWMA, testSpeed);
  analogWrite(motorPWMB, testSpeed);
  delay(testDuration);
  stopMotorDrivers();
}

void testSpin(bool spinLeft) {
  stopMotorDrivers();
  if (spinLeft) {
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
  } else {
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
  }
  analogWrite(motorPWMA, testSpeed);
  analogWrite(motorPWMB, testSpeed);
  delay(testDuration);
  stopMotorDrivers();
}

void stopMotorDrivers() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
  analogWrite(motorPWMA, 0);
  analogWrite(motorPWMB, 0);
}

// ============ ESC/DShot Functions ============
// These are EXACT copies from your working standalone code

void checkSerial() {
    if (Serial1.available()) {
        String cmd = Serial1.readStringUntil('\n');
        cmd.trim();  // remove \r, \n, spaces

        Serial1.print("Got command: '");
        Serial1.print(cmd);
        Serial1.println("'");

        if (cmd.equalsIgnoreCase("start")) {
            sweeping = true;
            Serial1.println("Sweep STARTED");
        } 
        else if (cmd.equalsIgnoreCase("stop")) {
            sweeping = false;
            motorval[0] = 0.0;
            motorval[1] = 0.0;
            Serial1.println("Sweep STOPPED, motors OFF");
        } 
        else if (cmd.equalsIgnoreCase("m")) {
            escMode = false;
            motorval[0] = 0.0;
            motorval[1] = 0.0;
            sweeping = false;
            Serial1.println("Switched to Motor Driver Mode");
        }
        else if (cmd.startsWith("M1=")) {
            float val = cmd.substring(3).toFloat();
            if (val >= 0.0 && val <= 1.0) {
                sweeping = false;
                motorval[0] = val;
                Serial1.print("Motor 1 set to: ");
                Serial1.println(val, 2);
            }
        }
        else if (cmd.startsWith("M2=")) {
            float val = cmd.substring(3).toFloat();
            if (val >= 0.0 && val <= 1.0) {
                sweeping = false;
                motorval[1] = val;
                Serial1.print("Motor 2 set to: ");
                Serial1.println(val, 2);
            }
        }
        else if (cmd.startsWith("both=")) {
            float val = cmd.substring(5).toFloat();
            if (val >= 0.0 && val <= 1.0) {
                sweeping = false;
                motorval[0] = val;
                motorval[1] = val;
                Serial1.print("Both motors set to: ");
                Serial1.println(val, 2);
            }
        }
        else {
            Serial1.println("Invalid input. Use: start / stop / M1=val / M2=val / both=val");
        }
    }
}

static void run(const uint32_t usec)
{
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
        } 
        else {
            // Manual mode → just resend last set values
            Serial1.print("Manual throttle: M1=");
            Serial1.print(motorval[0], 2);
            Serial1.print(" M2=");
            Serial1.println(motorval[1], 2);
        }

        // Send both motors
        dshot.write(motorval);
    }
}