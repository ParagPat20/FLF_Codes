// Arduino Line Following Robot - Advanced 2D Control System
// Dynamic 2D error calculation using curved sensor array geometry
// Curvature-aware PID control mixing lateral and heading errors
// Adaptive speed control for optimal cornering performance
// Enhanced edge sensor memory for sharp turn recovery
// CURVED SENSOR ARRAY OPTIMIZATION - Enhanced for arc-shaped sensor layout

#include <Arduino.h>
#include <dshot_stm32f4.h>
#include <HardwareSerial.h>
#include <vector>

// Forward declarations of structs
struct PoseError {
  float e_lat_mm;    // Lateral error in mm (positive = line to the right)
  float e_yaw_rad;   // Heading error in radians (positive = robot pointing left of line)
  bool valid;        // True if error calculation is valid
};

HardwareSerial Serial1(USART1);
// ===== CURVED SENSOR ARRAY GEOMETRY =====
// 7-sensor array: L3, L2, L1, M0(center), R1, R2, R3
const uint8_t sensorPins[7] = { PA4, PA5, PA6, PA0, PA1, PA2, PA3 };
//ESC pins
static const uint8_t R_THRUSTER = PB8;  // Motor 1
static const uint8_t L_THRUSTER = PB7;  // Motor 2
static const uint32_t UPDATE_RATE = 1000;  // Hz (1 ms) - continuous ESC packets

static std::vector<uint8_t> escpins = {R_THRUSTER, L_THRUSTER};
static Stm32F4Dshot dshot;

// IRQ handlers need access to dshot variable
extern "C" void DMA1_Stream7_IRQHandler(void) { 
  extern Stm32F4Dshot dshot;
  dshot.handleDmaIrqStream1(); 
}
extern "C" void DMA1_Stream6_IRQHandler(void) { 
  extern Stm32F4Dshot dshot;
  dshot.handleDmaIrqStream2(); 
}

// Motor throttle values [0.0 – 1.0]
static float escval[2] = {0.0, 0.0};  

// Curved sensor array constants
#define CURVED_WEIGHT_MULTIPLIER 1.0
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
#define PWMA PB1
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

// ===== 2D ERROR CALCULATION CONSTANTS =====
#define LOOKAHEAD_DISTANCE 80.0  // mm - lookahead distance for curvature-aware control
#define CURVATURE_FACTOR 2.5     // Factor for adaptive speed control based on heading error
#define MIN_SPEED_FACTOR 0.4     // Minimum speed factor (40% of base speed)
#define MAX_SPEED_FACTOR 1.0     // Maximum speed factor (100% of base speed)

// ===== PID VARIABLES =====
// Fixed PID values optimized for curved array with 2D control
float Kp = 0.15;  // Proportional gain (increased for 2D control)
float Ki = 0.030; // Integral gain (increased for better tracking)
float Kd = 0.08;  // Derivative gain (increased for stability)
int baseSpeed = 140;  // Base speed (will be modulated by adaptive speed control)
int currentSpeed = 140;  // Current adaptive speed

// ===== CONTROL PARAMETERS =====
int MAX_CORRECTION = 400;  // Maximum steering correction
int ERROR_DEADBAND = 10;   // Deadband to reduce wobbling on straight paths

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
const int MAX_DOTTED_FORWARD_STEPS = 40;          // Maximum steps to move forward before giving up (prevents going too far off course)
const unsigned long DOTTED_LINE_TIMEOUT = 800;    // 800ms timeout - if no line found within this time, exit dotted line mode

// ===== SMART TURN VARIABLES =====
bool leftEdgeDetected = false;    // Sensor 0 detected black
bool rightEdgeDetected = false;   // Sensor 7 detected black
unsigned long leftEdgeTime = 0;   // When left edge was last detected
unsigned long rightEdgeTime = 0;  // When right edge was last detected
int lastTurnDirection = 0;        // -1 = left, 1 = right, 0 = straight
bool inRecoveryMode = false;      // Currently trying to recover line
unsigned long recoveryStartTime = 0;

// ===== ROBOT STATE VARIABLES =====
bool robotRunning = true;  // Robot running state

// ===== 2D ERROR CALCULATION FUNCTIONS =====
PoseError calculate2DError() {
  PoseError result = {0.0, 0.0, false};
  
  float totalWeight = 0;
  float weightedX = 0;  // Forward direction (mm)
  float weightedY = 0;  // Lateral direction (mm) 
  int activeCount = 0;
  
  // Read all 7 sensors and calculate weighted centroid in 2D
  for (int i = 0; i < 7; i++) {
    int value = analogRead(sensorPins[i]);
    
    // Check if sensor detects black line
    if (value < sensorThresholds[i]) {
      activeCount++;
      
      // Convert polar coordinates (distance, angle) to Cartesian (x, y)
      float angleRad = sensorGeometry[i].angle * PI / 180.0;
      float x = sensorGeometry[i].distance * cos(angleRad);  // Forward
      float y = sensorGeometry[i].distance * sin(angleRad);  // Lateral (+ = left)
      
      // Apply curved array weighting
      float weight = sensorGeometry[i].weight;
      if (i == 0 || i == 6) {  // Outer sensors
        weight *= CURVED_WEIGHT_MULTIPLIER * 1.5;
      } else if (i == 1 || i == 5) {  // Second outer
        weight *= CURVED_WEIGHT_MULTIPLIER * 1.2;
      } else if (i == 2 || i == 4) {  // Inner sensors
        weight *= CURVED_WEIGHT_MULTIPLIER;
      }
      
      // Accumulate weighted position
      weightedX += x * weight;
      weightedY += y * weight;
      totalWeight += weight;
      
      // Update edge detection for recovery mode
      if (i == 0 || i == 1) {
        leftEdgeDetected = true;
        leftEdgeTime = millis();
      }
      if (i == 5 || i == 6) {
        rightEdgeDetected = true;
        rightEdgeTime = millis();
      }
    }
  }
  
  // Calculate errors if we have valid sensor data
  if (activeCount > 0 && totalWeight > 0) {
    // Calculate weighted centroid
    float centroidX = weightedX / totalWeight;
    float centroidY = weightedY / totalWeight;
    
    // Lateral error: positive when line is to the right of robot center
    result.e_lat_mm = -centroidY;  // Negative because sensor Y+ is left, but we want line-right to be positive
    
    // Heading error: angle between robot heading and line direction
    // Use atan2 to get angle from robot center to line centroid
    result.e_yaw_rad = atan2(centroidY, centroidX + 50.0);  // +50mm accounts for sensor array being ahead of robot center
    
    result.valid = true;
  }
  
  return result;
}

// Legacy function for compatibility (now uses 2D calculation)
float readLinePosition() {

  // Use 2D error calculation and convert to legacy position format
  PoseError poseError = calculate2DError();
  
  if (poseError.valid) {
    // Convert lateral error back to position scale (0-6000)
    // Lateral error of 0 = center position (3000)
    // Positive lateral error (line to right) = position > 3000
    // Negative lateral error (line to left) = position < 3000
    float position = CURVED_ARRAY_CENTER - (poseError.e_lat_mm * 10.0);  // Scale factor for compatibility
    return constrain(position, 0, 6000);
  } else {
    return -1;  // No line detected
  }
}

// ===== ADAPTIVE SPEED CONTROL FUNCTIONS =====
void updateAdaptiveSpeed(float headingError) {
  // Calculate speed reduction factor based on heading error magnitude
  float absHeadingError = fabs(headingError);
  float speedFactor = 1.0 / (1.0 + CURVATURE_FACTOR * absHeadingError);
  
  // Constrain speed factor to reasonable range
  speedFactor = constrain(speedFactor, MIN_SPEED_FACTOR, MAX_SPEED_FACTOR);
  
  // Update current speed
  currentSpeed = (int)(baseSpeed * speedFactor);
}

// ===== CURVATURE-AWARE PID CONTROL =====
float calculateMixedError(const PoseError& poseError) {
  // Mix lateral and heading errors with lookahead distance
  // error = e_lat_mm + L * e_yaw_rad
  return poseError.e_lat_mm + (LOOKAHEAD_DISTANCE * poseError.e_yaw_rad * 180.0 / PI); // Convert rad to degrees for scaling
}

void stopMotors() {
  // Stop motors and set escval to 0.0 for complete stop
  escval[0] = 0.0;  // Right thruster (stop)
  escval[1] = 0.0;  // Left thruster (stop)
  leftMotor(0);
  rightMotor(0);
}

void sharpLeftTurn() {
  // Sharp left turn - set escval for aggressive turning (effective range 0.4-0.9)
  escval[0] = 0.9;  // Right thruster (high forward)
  escval[1] = 0.4;  // Left thruster (low forward)
  leftMotor(-200);  // Maximum reverse left motor
  rightMotor(200);  // Maximum forward right motor
}

void sharpRightTurn() {
  // Sharp right turn - set escval for aggressive turning (effective range 0.4-0.9)
  escval[0] = 0.4;  // Right thruster (low forward)
  escval[1] = 0.9;  // Left thruster (high forward)
  leftMotor(200);    // Maximum forward left motor
  rightMotor(-200);  // Maximum reverse right motor
}

void fastLeftSearch() {
  // Fast left search - set escval for quick search pattern (effective range 0.4-0.9)
  escval[0] = 0.8;  // Right thruster (high forward)
  escval[1] = 0.5;  // Left thruster (moderate forward)
  leftMotor(50);    // Slow forward left
  rightMotor(200);  // Fast forward right
}

void fastRightSearch() {
  // Fast right search - set escval for quick search pattern (effective range 0.4-0.9)
  escval[0] = 0.5;  // Right thruster (moderate forward)
  escval[1] = 0.8;  // Left thruster (high forward)
  leftMotor(200);  // Fast forward left
  rightMotor(50);  // Slow forward right
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
  // If robot is stopped, just stop motors and return
  if(!robotRunning) {
    escval[0] = 0.0;  // Right thruster (stop)
    escval[1] = 0.0;  // Left thruster (stop)
    stopMotors();
    return;
  }
  
  unsigned long currentTime = millis();

  // Read sensors and calculate 2D error at regular intervals
  PoseError poseError = {0.0, 0.0, false};
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentTime;

    // Get 2D pose error from curved sensor array
    poseError = calculate2DError();

    if (poseError.valid) {
      // Line detected - update position for legacy compatibility
      currentPosition = CURVED_ARRAY_CENTER - (poseError.e_lat_mm * 10.0);
      currentPosition = constrain(currentPosition, 0, 6000);
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

      // Use new 2D error calculation for curvature-aware control
      if (poseError.valid) {
        // Calculate mixed error combining lateral and heading components
        error = calculateMixedError(poseError);
        
        // Update adaptive speed based on heading error (curvature)
        updateAdaptiveSpeed(poseError.e_yaw_rad);
        
        // Apply deadband to reduce jitter from small errors
        if (abs(error) < ERROR_DEADBAND) {
          error = 0;     // Ignore very small errors
          integral = 0;  // Reset integral when centered
        }
        
        // Update turn direction memory based on lateral error
        if (poseError.e_lat_mm < -20.0) lastTurnDirection = -1;  // Line on left
        else if (poseError.e_lat_mm > 20.0) lastTurnDirection = 1;   // Line on right
      } else {
        // Fallback to legacy position-based error if 2D calculation fails
        error = CURVED_ARRAY_CENTER - currentPosition;
        currentSpeed = baseSpeed;  // Use base speed as fallback
      }

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

      // PID control with adaptive speed based on curvature
      int leftSpeed = currentSpeed - correction;
      int rightSpeed = currentSpeed + correction;

      // Update escval for ESC control based on error magnitude (effective range 0.4-0.9)
      float errorMagnitude = abs(error);
      if (errorMagnitude < ERROR_DEADBAND) {
        // Small error - smooth forward movement
        escval[0] = 0.5;  // Right thruster (moderate forward)
        escval[1] = 0.5;  // Left thruster (moderate forward)
      } else if (errorMagnitude < 100) {
        // Small to medium error - quick adjustments
        float baseThrottle = 0.6;
        float adjustment = correction / 1200.0;  // Scale correction to effective throttle range
        escval[0] = constrain(baseThrottle - adjustment, 0.4, 0.9);  // Right thruster
        escval[1] = constrain(baseThrottle + adjustment, 0.4, 0.9);  // Left thruster
      } else {
        // Large error - more aggressive correction
        float baseThrottle = 0.7;
        float adjustment = correction / 1000.0;  // Scale correction to effective throttle range
        escval[0] = constrain(baseThrottle - adjustment, 0.4, 0.9);  // Right thruster
        escval[1] = constrain(baseThrottle + adjustment, 0.4, 0.9);  // Left thruster
      }

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



      // Check if we should attempt recovery based on navigation data or edge sensor memory
      unsigned long timeSinceLoss = currentTime - lineDetectedTime;

      if (timeSinceLoss < 2000) {  // Within 2000ms window for recovery
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
        // Set escval to 0.0 during recovery timeout
        escval[0] = 0.0;  // Right thruster (stop)
        escval[1] = 0.0;  // Left thruster (stop)
        stopMotors();
        integral = 0;
        currentPosition = CURVED_ARRAY_CENTER;  // Reset to center position for curved array
        leftEdgeDetected = false;
        rightEdgeDetected = false;
      }
    }
  }
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
  }
  
  // STEP 2: Handle behavior while in dotted line mode
  if (inDottedLineMode) {
    // EXIT CONDITION 1: Check if we found line again
    // This is the success case - we found the line after the gap
    if (lineDetected) {
      inDottedLineMode = false;                 // Exit dotted line mode
      return;                                   // Skip normal recovery mode
    }
    
    // EXIT CONDITION 2: Check if we've been in dotted line mode too long
    // This prevents the robot from going too far if there's no line ahead
    if ((currentTime - dottedLineStartTime) > DOTTED_LINE_TIMEOUT) {
      inDottedLineMode = false;                 // Exit dotted line mode
      return;                                   // Fall back to normal recovery
    }
    
    // EXIT CONDITION 3: Check if we've moved forward enough steps
    // This prevents the robot from going too far off course
    if (dottedLineForwardCount >= MAX_DOTTED_FORWARD_STEPS) {
      inDottedLineMode = false;                 // Exit dotted line mode
      return;                                   // Fall back to normal recovery
    }
    
    // STEP 3: Continue forward movement in dotted line mode
    dottedLineForwardCount++;                   // Increment the step counter
    
    // Use last known position to maintain direction
    // This keeps the robot moving in the same direction it was going before losing the line
    float lastKnownError = CURVED_ARRAY_CENTER - currentPosition;
    float correction = constrain(lastKnownError * 0.5, -200, 200);  // Reduced correction (50% of normal)
    
    // Set escval for steady forward movement in dotted line mode - keep both at 0.7
    escval[0] = 0.7;  // Right thruster (steady forward)
    escval[1] = 0.7;  // Left thruster (steady forward)
    
    // Set forward speed slightly above base for controlled movement
    int forwardSpeed = baseSpeed + 20;       // Base speed + 20 for steady forward motion
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
  dshot.begin(escpins);

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

  // Brief startup delay
  delay(1000);
}


void loop() {
  const auto usec = micros();
  
  // ESC update runs independently at its own rate
  run(usec);
  
}


static void run(const uint32_t usec)
{
    static uint32_t prev;

    if (usec - prev > 1000000 / UPDATE_RATE) {
        prev = usec;
        // Send both motors
        executeLineFollowing();
        dshot.write(escval);
    }
}