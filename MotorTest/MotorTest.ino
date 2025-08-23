#include <HardwareSerial.h>

// HardwareSerial instance on PA9 (TX) / PA10 (RX)
HardwareSerial Serial1(USART1);

// Motor driver pins (from STM32_BPW34_TB6612.ino)
#define PWMA PB13
#define AIN2 PB14
#define AIN1 PB15
#define BIN1 PA8
#define BIN2 PB3
#define PWMB PB4

// Motor control functions
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

void setup() {
  // Initialize motor pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Start serial communication
  Serial1.begin(115200);
  
  // Initial stop
  stopMotors();
  
  Serial1.println("=== MOTOR TEST STARTED ===");
  Serial1.println("Testing motors forward and backward...");
  delay(2000);
}

void loop() {
  // Test sequence with serial output
  
  // 1. Test left motor forward
  Serial1.println("Testing LEFT MOTOR FORWARD (Speed: 150)");
  leftMotor(150);
  rightMotor(0);
  delay(2000);
  stopMotors();
  Serial1.println("Left motor forward test complete");
  delay(500);

  // 2. Test left motor backward
  Serial1.println("Testing LEFT MOTOR BACKWARD (Speed: -150)");
  leftMotor(-150);
  rightMotor(0);
  delay(2000);
  stopMotors();
  Serial1.println("Left motor backward test complete");
  delay(500);

  // 3. Test right motor forward
  Serial1.println("Testing RIGHT MOTOR FORWARD (Speed: 150)");
  leftMotor(0);
  rightMotor(150);
  delay(2000);
  stopMotors();
  Serial1.println("Right motor forward test complete");
  delay(500);

  // 4. Test right motor backward
  Serial1.println("Testing RIGHT MOTOR BACKWARD (Speed: -150)");
  leftMotor(0);
  rightMotor(-150);
  delay(2000);
  stopMotors();
  Serial1.println("Right motor backward test complete");
  delay(500);

  // 5. Test both motors forward
  Serial1.println("Testing BOTH MOTORS FORWARD (Speed: 150)");
  leftMotor(150);
  rightMotor(150);
  delay(2000);
  stopMotors();
  Serial1.println("Both motors forward test complete");
  delay(500);

  // 6. Test both motors backward
  Serial1.println("Testing BOTH MOTORS BACKWARD (Speed: -150)");
  leftMotor(-150);
  rightMotor(-150);
  delay(2000);
  stopMotors();
  Serial1.println("Both motors backward test complete");
  delay(500);

  // 7. Test turning left (left backward, right forward)
  Serial1.println("Testing LEFT TURN (Left: -150, Right: 150)");
  leftMotor(-150);
  rightMotor(150);
  delay(2000);
  stopMotors();
  Serial1.println("Left turn test complete");
  delay(500);

  // 8. Test turning right (left forward, right backward)
  Serial1.println("Testing RIGHT TURN (Left: 150, Right: -150)");
  leftMotor(150);
  rightMotor(-150);
  delay(2000);
  stopMotors();
  Serial1.println("Right turn test complete");
  delay(500);

  Serial1.println("=== ALL MOTOR TESTS COMPLETE ===");
  Serial1.println("Waiting 5 seconds before repeating...");
  delay(5000);
}
