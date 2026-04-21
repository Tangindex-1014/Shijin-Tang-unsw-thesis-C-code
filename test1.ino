const int sensorPin = A0;

// =========================
// 可调参数区
// =========================

// L298N 引脚
const int ENA = 5;
const int ENB = 6;
const int IN1 = 7;
const int IN2 = 8;
const int IN3 = 9;
const int IN4 = 10;

// 基准压力读取设置
const unsigned long baselineReadTimeMs = 3000;  // 前3秒读取压力求平均
const int pressureRange = 80;                   // 上限 = 下限 + 30

// 电机速度
const int motorSpeed = 255;

// 总往返次数
const int maxCycles = 10;

// 串口输出频率
const unsigned long plotInterval = 50;
unsigned long lastPlotTime = 0;

// 趋势检测时间间隔
const unsigned long trendCheckInterval = 150;
unsigned long lastTrendCheckTime = 0;

// 压力变化容忍范围，防止噪声误判
const int changeTolerance = 2;

// 平滑采样参数
const int smoothSamples = 6;
const int sampleDelayMs = 1;

// 启动延时
const int startupDelayMs = 1000;


// =========================
// 运行变量区
// =========================

int motor1Plot = 0;
int motor2Plot = 0;

int lastTrendSensorValue = 0;

// 自动计算出的压力上下限
int lowThreshold = 0;
int highThreshold = 0;

// 已完成往返次数
int completedCycles = 0;

// 当前是否正在运行一次测试
bool isRunning = false;

enum MotorState {
  WAIT_STATE,
  FORWARD_STATE,
  REVERSE_STATE,
  STOP_STATE
};

MotorState motorState = WAIT_STATE;


// =========================
// 功能函数区
// =========================

int readSensorSmooth() {
  long sum = 0;
  for (int i = 0; i < smoothSamples; i++) {
    sum += analogRead(sensorPin);
    delay(sampleDelayMs);
  }
  return sum / smoothSamples;
}

void motorForward(int speedVal) {
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  motor1Plot = speedVal;
  motor2Plot = speedVal;
}

void motorReverse(int speedVal) {
  analogWrite(ENA, speedVal);
  analogWrite(ENB, speedVal);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  motor1Plot = -speedVal;
  motor2Plot = -speedVal;
}

void motorStop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  motor1Plot = 0;
  motor2Plot = 0;
}

int calculateBaselinePressure(unsigned long durationMs) {
  unsigned long startTime = millis();
  long total = 0;
  long count = 0;

  while (millis() - startTime < durationMs) {
    int value = readSensorSmooth();
    total += value;
    count++;
  }

  if (count == 0) return 0;
  return total / count;
}

void startNewTest() {
  completedCycles = 0;
  motor1Plot = 0;
  motor2Plot = 0;

  Serial.println("START_BASELINE");

  lowThreshold = calculateBaselinePressure(baselineReadTimeMs);
  highThreshold = lowThreshold + pressureRange;

  Serial.println("Time_ms,Pressure,LowLimit,HighLimit,Motor1,Motor2,Cycle");

  motorForward(motorSpeed);
  motorState = FORWARD_STATE;
  isRunning = true;

  lastTrendSensorValue = readSensorSmooth();
  lastTrendCheckTime = millis();
  lastPlotTime = millis();
}


// =========================
// 初始化
// =========================

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  delay(startupDelayMs);

  motorStop();
  motorState = WAIT_STATE;
  isRunning = false;

  Serial.println("WAITING_FOR_START");
}


// =========================
// 主循环
// =========================

void loop() {
  // 等待 MATLAB 发来启动命令 S
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'S') {
      motorStop();
      motorState = WAIT_STATE;
      isRunning = false;
      startNewTest();
    }
  }

  // 如果当前没有运行测试，就什么都不做
  if (!isRunning) {
    return;
  }

  int sensorValue = readSensorSmooth();

  if (motorState != STOP_STATE) {

    // 基本阈值控制
    if (motorState == FORWARD_STATE && sensorValue > highThreshold) {
      motorReverse(motorSpeed);
      motorState = REVERSE_STATE;
      lastTrendSensorValue = sensorValue;
      lastTrendCheckTime = millis();
    }
    else if (motorState == REVERSE_STATE && sensorValue <= lowThreshold) {
      completedCycles++;

      if (completedCycles >= maxCycles) {
        motorStop();
        motorState = STOP_STATE;
        isRunning = false;
        Serial.println("FINISHED");
        Serial.println("WAITING_FOR_START");
        return;
      }
      else {
        motorForward(motorSpeed);
        motorState = FORWARD_STATE;
        lastTrendSensorValue = sensorValue;
        lastTrendCheckTime = millis();
      }
    }

    // 趋势检测
    if (motorState != STOP_STATE && millis() - lastTrendCheckTime >= trendCheckInterval) {
      int delta = sensorValue - lastTrendSensorValue;

      if (motorState == FORWARD_STATE && delta < -changeTolerance) {
        motorReverse(motorSpeed);
        motorState = REVERSE_STATE;
      }
      else if (motorState == REVERSE_STATE && delta > changeTolerance) {
        motorForward(motorSpeed);
        motorState = FORWARD_STATE;
      }

      lastTrendSensorValue = sensorValue;
      lastTrendCheckTime = millis();
    }
  }

  // 串口输出给 MATLAB
  if (millis() - lastPlotTime >= plotInterval) {
    lastPlotTime = millis();

    Serial.print(millis());
    Serial.print(',');

    Serial.print(sensorValue);
    Serial.print(',');

    Serial.print(lowThreshold);
    Serial.print(',');

    Serial.print(highThreshold);
    Serial.print(',');

    Serial.print(motor1Plot);
    Serial.print(',');

    Serial.print(motor2Plot);
    Serial.print(',');

    Serial.println(completedCycles);
  }
}