/*
 * 2026-3-19
 * Author: Bingle
 * Agile precise obstacle avoidance car (complete version - multitasking upgrade)
 * Includes: straight-line PID hold + fast radar background scanning + MPU6050 closed-loop PID steering
 */

#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

// ================= Pin Definitions =================
// Motor pins
#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_AIN_1 7
#define PIN_Motor_BIN_1 8
#define PIN_Motor_STBY 3

// Ultrasonic and servo pins
#define TRIG_PIN 13
#define ECHO_PIN 12
#define SERVO_PIN 10

// ================= Global objects ================= 
MPU6050 mpu;
Servo radarServo;

// ================= Motion and control parameters =================
// Straight-line PID parameters
float yaw = 0;
float yaw_target = 0;
unsigned long lastTime = 0;
const float straight_Kp = 15.0;  // straight correction gain
const int BaseSpeed = 120;       // base straight speed
const int MaxSpeed = 255;

// Turning PID parameters
const float turn_Kp = 4.0;
const float turn_Ki = 0.05;
const float turn_Kd = 0.5;
const int minTurnSpeed = 85;     // minimum start speed to overcome static friction
const int maxTurnSpeed = 200;    // maximum turning speed

// ================= Radar scanning and state machine params =================
const int MAX_DISTANCE = 60;     // radar max range 50cm
const int SafeDistance = 35;     // obstacle avoidance trigger distance 15cm
const int NUM_ANGLES = 5;     // radar max view 50cm
int scanAngles[NUM_ANGLES] = {0, 45, 90, 135, 180}; 
unsigned int distances[NUM_ANGLES]; 

// Non-blocking multitask control variables
unsigned long lastRadarTime = 0;
const int scanInterval = 80;     // servo step interval (ms)
int currentScanIndex = 2;         // initially points to forward (scanAngles[2] = 90°)
int scanDirection = 1;            // scan direction: 1 right, -1 left

// ================= Initialization =================
void setup() {
  Wire.begin();

  // 1. initialize motors
  pinMode(PIN_Motor_PWMA, OUTPUT); pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT); pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
  digitalWrite(PIN_Motor_STBY, HIGH); // wake up motor driver

  // 2. initialize ultrasonic and servo
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  radarServo.attach(SERVO_PIN);
  radarServo.write(90); // servo center forward
  
  // initialize distances to avoid startup false zero and deadlock
  for(int i = 0; i < NUM_ANGLES; i++) {
    distances[i] = MAX_DISTANCE;
  }

  // 3. Initialize and calibrate MPU6050
  mpu.initialize();
  delay(2000); // Wait stationary for calibration
  lastTime = millis();
  yaw_target = 0;
}

// ================= Core sensor functions =================
// Update current yaw angle
void updateYaw() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  
  // Prevent large dt caused by waits (e.g. ultrasonic timeout) from injecting spikes
  if (dt > 0.5) dt = 0.001; 
  
  int16_t gz = mpu.getRotationZ();
  float gyroz = gz / 131.0; 
  if (abs(gyroz) > 1.0) { // deadzone filter for static drift
    yaw += gyroz * dt;
  }
}

unsigned int getDistance() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Originally 30000, now 4500.
  // 4500us is enough for about 75cm, covering our 60cm need.
  unsigned long pulseTime = pulseIn(ECHO_PIN, HIGH, 4500); 
  
  if (pulseTime == 0) return MAX_DISTANCE; 
  
  unsigned int dist = (unsigned int)(pulseTime / 58.0);
  return (dist > MAX_DISTANCE) ? MAX_DISTANCE : dist;
}

// ================= Background radar task =================
void updateRadarTask() {
  unsigned long now = millis();
  
  // execute action only when interval passed
  if (now - lastRadarTime >= scanInterval) {
    lastRadarTime = now;

    // 1. read current direction distance and store
    distances[currentScanIndex] = getDistance();

    // 2. move to next angle
    currentScanIndex += scanDirection;
    
    // reverse scan direction at edges
    if (currentScanIndex >= NUM_ANGLES - 1 || currentScanIndex <= 0) {
      scanDirection = -scanDirection; 
    }

    // 3. command servo to new angle
    radarServo.write(scanAngles[currentScanIndex]);
  }
}

// Instant decision: judge directly from background scan data
int evaluateBestDirection() {
  int maxDistance = 0;
  int bestAngle = 90; 
  bool allBlocked = true; 

  for (int i = 0; i < NUM_ANGLES; i++) {
    if (distances[i] > SafeDistance) allBlocked = false; 

    if (distances[i] > maxDistance) {
      maxDistance = distances[i];
      bestAngle = scanAngles[i];
    } else if (distances[i] == maxDistance) {
      // if distance ties, choose path closest to forward
      if (abs(scanAngles[i] - 90) < abs(bestAngle - 90)) {
        bestAngle = scanAngles[i];
      }
    }
  }

  if (allBlocked) return 999; 
  return bestAngle - 90; // return relative turn angle from chassis orientation
}


// ================= Basic motion control =================
void stopCar() {
  analogWrite(PIN_Motor_PWMA, 0);
  analogWrite(PIN_Motor_PWMB, 0);
}

void moveBackward(int speed, int time_ms) {
  digitalWrite(PIN_Motor_AIN_1, LOW); 
  digitalWrite(PIN_Motor_BIN_1, LOW); 
  analogWrite(PIN_Motor_PWMA, speed);
  analogWrite(PIN_Motor_PWMB, speed);
  delay(time_ms);
  stopCar();
}

// Straight-line PID posture hold
void moveStraightPID() {
  updateYaw();
  float error = yaw - yaw_target; 
  
  int speed_L = BaseSpeed - (error * straight_Kp); // assume left yaw is positive
  int speed_R = BaseSpeed + (error * straight_Kp);

  // constrain speed to valid range
  speed_L = constrain(speed_L, 0, MaxSpeed);
  speed_R = constrain(speed_R, 0, MaxSpeed);

  digitalWrite(PIN_Motor_AIN_1, HIGH); 
  analogWrite(PIN_Motor_PWMA, speed_L);
  digitalWrite(PIN_Motor_BIN_1, HIGH); 
  analogWrite(PIN_Motor_PWMB, speed_R);
}

// Precise PID rotation (core)
void rotateRelativePID(float targetAngle) {
  updateYaw();
  float initialYaw = yaw; 
  float error = targetAngle; 
  float last_error = error;
  float integral = 0;
  unsigned long pidLastTime = millis();

  // continue adjustment while error > 1.5 degrees
  while (abs(error) > 1.5) {
    updateYaw();
    float angleTurned = yaw - initialYaw;
    error = targetAngle - angleTurned;

    unsigned long now = millis();
    float dt = (now - pidLastTime) / 1000.0;
    if (dt <= 0) dt = 0.001;
    pidLastTime = now;

    integral += error * dt;
    float derivative = (error - last_error) / dt;
    last_error = error;

    float output = (turn_Kp * error) + (turn_Ki * integral) + (turn_Kd * derivative);
    int motorSpeed = abs(output);
    
    if (motorSpeed < minTurnSpeed) motorSpeed = minTurnSpeed; 
    if (motorSpeed > maxTurnSpeed) motorSpeed = maxTurnSpeed;

    if (output > 0) { // turn right
      digitalWrite(PIN_Motor_AIN_1, HIGH); analogWrite(PIN_Motor_PWMA, motorSpeed);
      digitalWrite(PIN_Motor_BIN_1, LOW);  analogWrite(PIN_Motor_PWMB, motorSpeed);
    } else { // turn left
      digitalWrite(PIN_Motor_AIN_1, LOW);  analogWrite(PIN_Motor_PWMA, motorSpeed);
      digitalWrite(PIN_Motor_BIN_1, HIGH); analogWrite(PIN_Motor_PWMB, motorSpeed);
    }
    delay(10); 
  }

  stopCar();
  delay(150); // wait for chassis to settle
  
  // after turning, update straight PID heading lock to current heading
  updateYaw();
  yaw_target = yaw; 
}


// ================= New multitask main loop =================
// Add warning distance in global variables (farther than hard brake safe distance)
const int WarningDistance = 45;  // side obstacle warning distance 35cm

// ================= Dynamic correction multitask loop =================
void loop() {
  // Task 1: keep radar scanning in background (non-blocking)
  updateRadarTask();

  // Task 2: extract real-time data
  unsigned int rightDist = distances[1];     // 45° (front-right)
  unsigned int frontDist = distances[2];     // 90° (forward)
  unsigned int leftDist = distances[3];      // 135° (front-left)

  if (frontDist > SafeDistance) {
    // === core upgrade: dynamic heading correction (repulsion field) ===
    // compute repulsion from left/right based on warning distance
    int leftRepulsion = 0;
    if (leftDist < WarningDistance) leftRepulsion += (WarningDistance - leftDist);
    
    int rightRepulsion = 0;
    if (rightDist < WarningDistance) rightRepulsion += (WarningDistance - rightDist);

    // Apply repulsion difference to smoothly adjust PID heading target (yaw_target)
    // Note: step size (like 0.5) controls avoidance agility.
    // If it steers into obstacles, swap the += and -= below (depends on motor/MPU6050 orientation).
    if (leftRepulsion > rightRepulsion) {
        yaw_target -= 0.5; // left side danger, target heading shifts right
    } else if (rightRepulsion > leftRepulsion) {
        yaw_target += 0.5; // right side danger, target heading shifts left
    }

    // front is still safe; run straight PID with updated yaw_target
    // robot steers smoothly around side obstacles
    moveStraightPID();
    
  } else {
    // front obstacle detected: brake immediately
    stopCar();
    delay(100); 

    // instant turn decision
    int turnAngle = evaluateBestDirection(); 
    
    if (turnAngle == 999) {
      moveBackward(BaseSpeed, 600); 
      rotateRelativePID(180.0);    
    } 
    else if (turnAngle != 0) {
      rotateRelativePID((float)turnAngle);
    }
    
    // after turn complete, reset radar to forward
    currentScanIndex = 2;
    radarServo.write(90);
    delay(200); 
  }
  
  delay(10); 
}