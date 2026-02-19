/*
 * 2026-2-19
 * 敏捷精准避障小车 (完全体 - 多线程升级版)
 * 包含: 直线PID保持 + 极速雷达后台扫描 + MPU6050闭环PID转向
 */

#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

// ================= 引脚定义 =================
// 电机引脚
#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_AIN_1 7
#define PIN_Motor_BIN_1 8
#define PIN_Motor_STBY 3

// 超声波与舵机引脚
#define TRIG_PIN 13
#define ECHO_PIN 12
#define SERVO_PIN 10

// ================= 全局对象 =================
MPU6050 mpu;
Servo radarServo;

// ================= 运动与控制参数 =================
// 直行 PID 参数
float yaw = 0;
float yaw_target = 0;
unsigned long lastTime = 0;
const float straight_Kp = 15.0;  // 直行纠偏力度
const int BaseSpeed = 120;       // 基础直行速度
const int MaxSpeed = 255;

// 转向 PID 参数
const float turn_Kp = 4.0;
const float turn_Ki = 0.05;
const float turn_Kd = 0.5;
const int minTurnSpeed = 85;     // 克服静摩擦的最小启动速度
const int maxTurnSpeed = 200;    // 最大转向速度

// ================= 雷达扫描与状态机参数 =================
const int MAX_DISTANCE = 60;     // 雷达最远视野 50cm
const int SafeDistance = 35;     // 触发避障的危险距离 15cm
const int NUM_ANGLES = 7;
int scanAngles[NUM_ANGLES] = {0, 30, 60, 90, 120, 150, 180}; 
unsigned int distances[NUM_ANGLES]; 

// 非阻塞多线程控制变量
unsigned long lastRadarTime = 0;
const int scanInterval = 80;     // 舵机每步之间的间隔时间 (毫秒)
int currentScanIndex = 3;         // 初始指向正前方 (scanAngles[3] = 90度)
int scanDirection = 1;            // 扫描方向：1为向右扫，-1为向左扫

// ================= 初始化 =================
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // 1. 初始化电机
  pinMode(PIN_Motor_PWMA, OUTPUT); pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT); pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
  digitalWrite(PIN_Motor_STBY, HIGH); // 唤醒电机驱动

  // 2. 初始化超声波与舵机
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  radarServo.attach(SERVO_PIN);
  radarServo.write(90); // 舵机正前
  
  // 初始化距离数组，防止开局误判为 0 而直接死锁
  for(int i = 0; i < NUM_ANGLES; i++) {
    distances[i] = MAX_DISTANCE;
  }

  // 3. 初始化并校准 MPU6050
  Serial.println("初始化MPU6050，请保持静止...");
  mpu.initialize();
  delay(2000); // 必须静止等待校准
  lastTime = millis();
  yaw_target = 0;
  
  Serial.println("系统准备就绪，出发！");
}

// ================= 核心传感器函数 =================
// 更新当前偏航角 Yaw
void updateYaw() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  
  // 防止程序因等待(比如超声波超时)造成dt过大，进而引入突变误差
  if (dt > 0.5) dt = 0.001; 
  
  int16_t gz = mpu.getRotationZ();
  float gyroz = gz / 131.0; 
  if (abs(gyroz) > 1.0) { // 死区过滤静止漂移
    yaw += gyroz * dt;
  }
}

unsigned int getDistance() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // 原来是 30000，现在改为 4500。
  // 4500us 足够超声波测算 75cm 的距离，完美覆盖我们 60cm 的需求。
  unsigned long pulseTime = pulseIn(ECHO_PIN, HIGH, 4500); 
  
  if (pulseTime == 0) return MAX_DISTANCE; 
  
  unsigned int dist = (unsigned int)(pulseTime / 58.0);
  return (dist > MAX_DISTANCE) ? MAX_DISTANCE : dist;
}

// ================= 后台雷达任务 =================
void updateRadarTask() {
  unsigned long now = millis();
  
  // 如果经过的时间大于设定的间隔，才执行一次动作
  if (now - lastRadarTime >= scanInterval) {
    lastRadarTime = now;

    // 1. 获取当前指向的距离，并存入数组
    distances[currentScanIndex] = getDistance();

    // 2. 准备走向下一个角度
    currentScanIndex += scanDirection;
    
    // 如果扫到了边缘，就反转方向
    if (currentScanIndex >= NUM_ANGLES - 1 || currentScanIndex <= 0) {
      scanDirection = -scanDirection; 
    }

    // 3. 命令舵机移动到新角度
    radarServo.write(scanAngles[currentScanIndex]);
  }
}

// 瞬间决策：直接基于后台持续扫描获取的数据进行判断
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
      // 如果距离相同，选更接近正前方的那条路
      if (abs(scanAngles[i] - 90) < abs(bestAngle - 90)) {
        bestAngle = scanAngles[i];
      }
    }
  }

  if (allBlocked) return 999; 
  return bestAngle - 90; // 返回相对底盘的转向角度
}


// ================= 基础运动控制 =================
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

// 直线 PID 姿态保持
void moveStraightPID() {
  updateYaw();
  float error = yaw - yaw_target; 
  
  int speed_L = BaseSpeed - (error * straight_Kp); // 假设左偏偏航为正
  int speed_R = BaseSpeed + (error * straight_Kp);

  // 限制速度在合法范围内
  speed_L = constrain(speed_L, 0, MaxSpeed);
  speed_R = constrain(speed_R, 0, MaxSpeed);

  digitalWrite(PIN_Motor_AIN_1, HIGH); 
  analogWrite(PIN_Motor_PWMA, speed_L);
  digitalWrite(PIN_Motor_BIN_1, HIGH); 
  analogWrite(PIN_Motor_PWMB, speed_R);
}

// 精准 PID 转向 (核心)
void rotateRelativePID(float targetAngle) {
  updateYaw();
  float initialYaw = yaw; 
  float error = targetAngle; 
  float last_error = error;
  float integral = 0;
  unsigned long pidLastTime = millis();

  // 当误差大于 1.5 度时持续调整
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

    if (output > 0) { // 右转
      digitalWrite(PIN_Motor_AIN_1, HIGH); analogWrite(PIN_Motor_PWMA, motorSpeed);
      digitalWrite(PIN_Motor_BIN_1, LOW);  analogWrite(PIN_Motor_PWMB, motorSpeed);
    } else { // 左转
      digitalWrite(PIN_Motor_AIN_1, LOW);  analogWrite(PIN_Motor_PWMA, motorSpeed);
      digitalWrite(PIN_Motor_BIN_1, HIGH); analogWrite(PIN_Motor_PWMB, motorSpeed);
    }
    delay(10); 
  }

  stopCar();
  delay(150); // 等待车身完全停止晃动
  
  // 转向完成后，更新直线PID的锁定目标为当前车头方向
  updateYaw();
  yaw_target = yaw; 
}


// ================= 全新多线程主循环 =================
// 在全局变量区增加一个预警距离（比紧急刹车的SafeDistance要远）
const int WarningDistance = 45;  // 侧方障碍物预警距离 35cm

// ================= 动态修正多线程主循环 =================
void loop() {
  // 任务 1：让雷达在后台持续摇摆测距（非阻塞）
  updateRadarTask();

  // 任务 2：提取实时数据
  unsigned int rightDist2 = distances[1];     // 30度 (右前)
  unsigned int rightDist1 = distances[2];     // 60度 (右前偏正)
  unsigned int frontDist  = distances[3];     // 90度 (正前)
  unsigned int leftDist1  = distances[4];     // 120度 (左前偏正)
  unsigned int leftDist2  = distances[5];     // 150度 (左前)

  if (frontDist > SafeDistance) {
    // === 核心升级：动态航向修正 (排斥力场) ===
    // 计算左右两侧的受迫程度 (距离预警线越近，排斥力越大)
    int leftRepulsion = 0;
    if (leftDist1 < WarningDistance) leftRepulsion += (WarningDistance - leftDist1);
    if (leftDist2 < WarningDistance) leftRepulsion += (WarningDistance - leftDist2);
    
    int rightRepulsion = 0;
    if (rightDist1 < WarningDistance) rightRepulsion += (WarningDistance - rightDist1);
    if (rightDist2 < WarningDistance) rightRepulsion += (WarningDistance - rightDist2);

    // 根据两侧排斥力的差值，动态平滑地修改 PID 锁定的目标航向 (yaw_target)
    // 注意：每次循环加减的值(如 0.5)决定了避让的敏捷度。
    // 如果发现它向障碍物撞过去，请把下面的 += 和 -= 对调！(取决于你的电机和MPU6050安装方向)
    if (leftRepulsion > rightRepulsion) {
        yaw_target -= 0.5; // 左侧有危险，目标航向向右偏
    } else if (rightRepulsion > leftRepulsion) {
        yaw_target += 0.5; // 右侧有危险，目标航向向左偏
    }

    // 前方依然安全，带着动态更新的 yaw_target 开启直线 PID 狂奔
    // 小车会自动画出优美的弧线绕过侧方障碍物！
    moveStraightPID();
    
  } else {
    // 发现正前方障碍！立刻刹车（触发底线）
    stopCar();
    delay(100); 

    // 瞬间决策转向角度
    int turnAngle = evaluateBestDirection(); 
    Serial.print("前方受阻！瞬间决策转向角度: ");
    Serial.println(turnAngle);
    
    if (turnAngle == 999) {
      Serial.println("绝境！启动后退掉头");
      moveBackward(BaseSpeed, 600); 
      rotateRelativePID(180.0);    
    } 
    else if (turnAngle != 0) {
      rotateRelativePID((float)turnAngle);
    }
    
    // 转向完成后，重置雷达到正前方
    currentScanIndex = 3;
    radarServo.write(90);
    delay(200); 
  }
  
  delay(10); 
}