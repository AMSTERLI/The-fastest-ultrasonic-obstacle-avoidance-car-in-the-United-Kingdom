/*
 * 2026-3-20
 * Authors: Bingle Li
 * Agile precise obstacle avoidance smart car with high-frequency radar scanning,
 * adaptive acceleration control, timeout-based obstacle escape, and PID-based steering.
 */

#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

// ================= Pin Definitions =================
#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_AIN_1 7
#define PIN_Motor_BIN_1 8
#define PIN_Motor_STBY 3

#define TRIG_PIN 13
#define ECHO_PIN 12
#define SERVO_PIN 10

// ================= Global Objects =================
MPU6050 mpu;
Servo radarServo;

// ================= Motion and Control Parameters =================
float yaw = 0;
float yaw_target = 0;
unsigned long lastTime = 0;
const float straight_Kp = 15.0;  

// Speed PID parameters (Adaptive Cruise Control)
const float speed_Kp = 4.5;      
const float speed_Kd = 0.5;      
float speed_last_error = 0;
unsigned long speed_lastTime = 0;
const int MaxSpeed = 220;
const int MinForwardSpeed = 80;  

// ====== Smooth Acceleration Control ======
const float AccelRate = 100.0;   // Max PWM increase per second. Smaller = smoother acceleration
float currentForwardSpeed = 0;  // Current forward speed with smooth transition

// Steering PID parameters
const float turn_Kp = 4.0;
const float turn_Ki = 0.05;
const float turn_Kd = 0.2;
const int minTurnSpeed = 150;     
const int maxTurnSpeed = 220;    

// ================= Radar High-Frequency Scanning Parameters =================
const int MAX_DISTANCE = 60;     // Max radar vision range (cm)
const int SafeDistanceFront = 15; // Front safety threshold to trigger braking (cm)
const int SafeDistanceSide = 5;  // Side safety threshold for emergency stop (cm)
const int WarningDistance = 35;  // Warning distance for smooth lateral avoidance (cm)

const int NUM_ANGLES = 3;     
int scanAngles[NUM_ANGLES] = {145, 90, 35}; 
unsigned int distances[NUM_ANGLES]; 

unsigned long lastRadarTime = 0;
const int scanInterval = 120;    // Scan interval (ms)
int currentScanIndex = 1;        // Initial scan index pointing forward (90°)
int scanDirection = 1;           

// ================= Initialization =================

void setup() {
  Wire.begin();

  pinMode(PIN_Motor_PWMA, OUTPUT); pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT); pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
  digitalWrite(PIN_Motor_STBY, HIGH); 

  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  radarServo.attach(SERVO_PIN);
  radarServo.write(90); 
  
  for(int i = 0; i < NUM_ANGLES; i++) {
    distances[i] = MAX_DISTANCE;
  }

  mpu.initialize();
  delay(500); 
  lastTime = millis();
  yaw_target = 0;

  // Drive straight at max speed until gap detected
  while (getDistance() >= 35) {
    // moveStraightPID maintains gyro-based heading correction
    moveStraightPID(MaxSpeed); 
  }

  // Stop when gap is detected
  stopCar();
  delay(50); // Stabilize and eliminate momentum

  // Rotate 90 degrees
  rotateRelativePID(90.0);

  // Restore radar position and prepare for main loop
  radarServo.write(90);

  yaw_target = yaw; // Set current heading as straight-line reference
  lastTime = millis();
  speed_lastTime = millis();
  lastRadarTime = millis();
  speed_last_error = 0;
  currentForwardSpeed = 0; // Smooth acceleration controlled by main loop
  currentScanIndex = 1;
}

// ================= Core Sensors and Background Tasks =================
void updateYaw() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  
  if (dt > 0.5) dt = 0.001; 
  
  int16_t gz = mpu.getRotationZ();
  float gyroz = gz / 131.0; // Convert to degrees/sec
  if (abs(gyroz) > 1.0) { // Reject noise below 1 deg/sec
    yaw += gyroz * dt;
  }
}

unsigned int getDistance() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  unsigned long pulseTime = pulseIn(ECHO_PIN, HIGH, 4500); 
  
  if (pulseTime == 0) return MAX_DISTANCE; 
  
  unsigned int dist = (unsigned int)(pulseTime / 58.0);
  return (dist > MAX_DISTANCE) ? MAX_DISTANCE : dist;
}

void updateRadarTask() {
  unsigned long now = millis();
  
  if (now - lastRadarTime >= scanInterval) {
    lastRadarTime = now;
    distances[currentScanIndex] = getDistance();
    currentScanIndex += scanDirection;
    
    if (currentScanIndex >= NUM_ANGLES - 1 || currentScanIndex <= 0) {
      scanDirection = -scanDirection; 
    }
    radarServo.write(scanAngles[currentScanIndex]);
  }
}

// ================= Motion Control =================
void stopCar() {
  analogWrite(PIN_Motor_PWMA, 0);
  analogWrite(PIN_Motor_PWMB, 0);
}

void moveStraightPID(int currentBaseSpeed) {
  updateYaw();
  float error = yaw - yaw_target; 
  
  int speed_L = currentBaseSpeed - (error * straight_Kp); 
  int speed_R = currentBaseSpeed + (error * straight_Kp);

  speed_L = constrain(speed_L, 0, MaxSpeed);
  speed_R = constrain(speed_R, 0, MaxSpeed);

  digitalWrite(PIN_Motor_AIN_1, HIGH); 
  analogWrite(PIN_Motor_PWMA, speed_L);
  digitalWrite(PIN_Motor_BIN_1, HIGH); 
  analogWrite(PIN_Motor_PWMB, speed_R);
}

// Rotate to target angle with PID control and 2-second timeout protection
void rotateRelativePID(float targetAngle) {
  updateYaw();
  float initialYaw = yaw; 
  float error = targetAngle; 
  float last_error = error;
  float integral = 0;
  unsigned long pidLastTime = millis();
  unsigned long turnStartTime = millis(); // Track turn start time

  while (abs(error) > 1.5) {
    
    // Timeout protection: if stuck for 2 seconds, reverse briefly to escape
    if (millis() - turnStartTime > 2000) {
      stopCar();
      delay(20); // Stabilize
      
      // Reverse motor briefly (AIN_1 and BIN_1 LOW = reverse)
      digitalWrite(PIN_Motor_AIN_1, LOW); 
      digitalWrite(PIN_Motor_BIN_1, LOW); 
      analogWrite(PIN_Motor_PWMA, 120); // Moderate reverse speed
      analogWrite(PIN_Motor_PWMB, 120);
      
      delay(200); // Reverse for 200ms
      stopCar();
      break; // Exit turn loop
    }

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

    if (output > 0) { // Turn right
      digitalWrite(PIN_Motor_AIN_1, HIGH); analogWrite(PIN_Motor_PWMA, motorSpeed);
      digitalWrite(PIN_Motor_BIN_1, LOW);  analogWrite(PIN_Motor_PWMB, motorSpeed);
    } else {          // Turn left
      digitalWrite(PIN_Motor_AIN_1, LOW);  analogWrite(PIN_Motor_PWMA, motorSpeed);
      digitalWrite(PIN_Motor_BIN_1, HIGH); analogWrite(PIN_Motor_PWMB, motorSpeed);
    }
    delay(5); 
  }

  stopCar();
  delay(150); 
  updateYaw();
  yaw_target = yaw; // Set current heading as new target to prevent body twist
}


// ================= Main Loop =================
void loop() {
  updateRadarTask();

  unsigned int leftDist = distances[0];     // Index 0 = 145°
  unsigned int frontDist = distances[1];     // Index 1 = 90°
  unsigned int rightDist  = distances[2];     // Index 2 = 35°

  bool isSafe = (frontDist > SafeDistanceFront);

  // Execute straight motion only when front is clear
  if (isSafe) {
    
    unsigned long now = millis();
    float dt = (now - speed_lastTime) / 1000.0;
    if (dt <= 0) dt = 0.001;
    speed_lastTime = now;

    float errorFront = frontDist - SafeDistanceFront;
    float errorRight = rightDist - SafeDistanceSide;
    float errorLeft  = leftDist - SafeDistanceSide;

    float distError = errorFront;
    if (errorRight < distError) distError = errorRight;
    if (errorLeft < distError) distError = errorLeft;

    float derivative = (distError - speed_last_error) / dt;
    speed_last_error = distError;

    float speedOutput = (distError * speed_Kp) + (derivative * speed_Kd);
    int targetSpeed = (int)speedOutput;

    if (targetSpeed > MaxSpeed) targetSpeed = MaxSpeed;
    if (targetSpeed < MinForwardSpeed) targetSpeed = MinForwardSpeed;

    // ====== Smooth Acceleration Core Logic ======
    if (targetSpeed > currentForwardSpeed) {
      currentForwardSpeed += AccelRate * dt; 
      if (currentForwardSpeed > targetSpeed) {
        currentForwardSpeed = targetSpeed; 
      }
    } else {
      currentForwardSpeed = targetSpeed; // Decelerate instantly for responsiveness
    }

    int leftRepulsion = 0;
    if (leftDist < WarningDistance) leftRepulsion += (WarningDistance - leftDist);
    
    int rightRepulsion = 0;
    if (rightDist < WarningDistance) rightRepulsion += (WarningDistance - rightDist);

    // Smooth lateral avoidance to prevent jerking
    float turnRate = 50.0; 
    if (leftRepulsion > rightRepulsion) {
        yaw_target -= turnRate * dt; 
    } else if (rightRepulsion > leftRepulsion) {
        yaw_target += turnRate * dt; 
    }

    // Apply smoothed speed to motor control
    moveStraightPID((int)currentForwardSpeed);
    
  } else {
    // Safety threshold triggered: emergency brake
    stopCar();
    currentForwardSpeed = 0; 
    delay(50); // Motor demagnetization and momentum elimination

    // Intelligent 360° scan to find optimal escape direction
    int bestAngle = 90;
    unsigned int maxDist = 0;

    // Move servo to 0° starting position
    radarServo.write(0);  
    delay(150); // Wait for servo movement

    // Scan from 0° to 180° in 30° increments
    for (int scanAng = 0; scanAng <= 180; scanAng += 30) {
      radarServo.write(scanAng);
      delay(50); // Wait for servo positioning 
      
      unsigned int currentDist = getDistance();
      
      if (currentDist > maxDist) {
        maxDist = currentDist;
        bestAngle = scanAng;
      }
    }
    float turnTarget=0;

    if(bestAngle>=160||bestAngle<=20){
      turnTarget = (float)0.5*(bestAngle - 90);
    }
    else if((bestAngle<160&&bestAngle>=120)||(bestAngle>20&&bestAngle<=60)){
      turnTarget = (float)0.65*(bestAngle - 90);
    }
    else{
      turnTarget = (float)0.6*(bestAngle - 90);
    }


    // Execute precise turn
    rotateRelativePID(turnTarget);
    
    // Reset radar position after escape
    radarServo.write(90);
    delay(120); // Wait for servo return to center
    
    distances[0] = getDistance(); 
    distances[1] = getDistance(); 
    distances[2] = getDistance(); 
    
    currentScanIndex = 1;
    lastRadarTime = millis(); 
    
    // Reset speed PID timers and errors
    speed_lastTime = millis();
    lastTime = millis(); 
    speed_last_error = 0;
  }
}