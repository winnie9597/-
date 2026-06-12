// =======================
// 5路 ADC 感測器
// =======================
const int sensorPins[5] = {26, 25, 34, 35, 32};
int w[5] = {-2, -1, 0, 1, 2};

// =======================
// 馬達 L298N
// =======================
const int motorL_IN1 = 19;
const int motorL_IN2 = 18;
const int motorL_PWM = 22;

const int motorR_IN1 = 27;
const int motorR_IN2 = 14;
const int motorR_PWM = 13;

// =======================
// PID（穩定參數）
// =======================
float Kp = 12;
float Kd = 3.5;

float error = 0;
float last_error = 0;
float derivative = 0;

// =======================
// 自動中心（重點升級）
// =======================
float autoCenter = 0;

// =======================
// 濾波
// =======================
float filt[5] = {0,0,0,0,0};
float alpha = 0.65;

// =======================
// 馬達補償（一定要調）
// =======================
int leftMotorTrim = 0;
int rightMotorTrim = 6;

// =======================
// 速度
// =======================
int maxSpeed = 180;

// =======================
// setup
// =======================
void setup() {

  Serial.begin(115200);

  for (int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  pinMode(motorL_IN1, OUTPUT);
  pinMode(motorL_IN2, OUTPUT);
  pinMode(motorR_IN1, OUTPUT);
  pinMode(motorR_IN2, OUTPUT);

  ledcAttachChannel(motorL_PWM, 1000, 8, 0);
  ledcAttachChannel(motorR_PWM, 1000, 8, 1);
}

// =======================
// 失線處理
// =======================
void searchLine() {

  if (last_error > 0) {
    digitalWrite(motorL_IN1, LOW);
    digitalWrite(motorL_IN2, HIGH);

    digitalWrite(motorR_IN1, HIGH);
    digitalWrite(motorR_IN2, LOW);
  } else {
    digitalWrite(motorL_IN1, HIGH);
    digitalWrite(motorL_IN2, LOW);

    digitalWrite(motorR_IN1, LOW);
    digitalWrite(motorR_IN2, HIGH);
  }

  ledcWriteChannel(0, 130);
  ledcWriteChannel(1, 130);
}

// =======================
// 主迴圈
// =======================
void loop() {

  // =======================
  // 1. ADC
  // =======================
  int raw[5];
  for (int i = 0; i < 5; i++) {
    raw[i] = analogRead(sensorPins[i]);
  }

  // =======================
  // 2. 加權
  // =======================
  float sum = 0;
  float position = 0;

  for (int i = 0; i < 5; i++) {

    int v = 4095 - raw[i];

    filt[i] = alpha * filt[i] + (1 - alpha) * v;

    sum += filt[i];
    position += filt[i] * w[i];
  }

  // =======================
  // 3. 失線
  // =======================
  if (sum < 50) {
    searchLine();
    return;
  }

  position = position / sum;

  // =======================
  // ⭐ 自動中心校正（核心升級）
  // =======================
  autoCenter = autoCenter * 0.995 + position * 0.005;

  error = position - autoCenter;

  // deadband（防抖）
  if (abs(error) < 0.04) error = 0;

  // =======================
  // 4. PID（穩定版）
  // =======================
  derivative = error - last_error;

  float turn = Kp * error + Kd * derivative;

  last_error = error;

  // =======================
  // ⭐ 非線性壓縮（防爆衝）
  // =======================
  turn = tanh(turn * 0.03) * 120;

  // =======================
  // 5. 速度控制（更穩）
  // =======================
  int speedAdjust = abs(error) * 8;
  int baseSpeed = constrain(maxSpeed - speedAdjust, 110, maxSpeed);

  // =======================
  // 6. 馬達輸出
  // =======================
  float turnScale = 0.5;

  int leftSpeed  = baseSpeed - turn * turnScale;
  int rightSpeed = baseSpeed + turn * turnScale;

  // 馬達補償（重點）
  leftSpeed += leftMotorTrim;
  rightSpeed += rightMotorTrim;

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // =======================
  // 限制差速（防甩頭）
  // =======================
  int diff = rightSpeed - leftSpeed;
  int maxDiff = 55;

  if (diff > maxDiff) rightSpeed = leftSpeed + maxDiff;
  if (diff < -maxDiff) leftSpeed = rightSpeed + maxDiff;

  // =======================
  // 左馬達
  // =======================
  if (leftSpeed >= 0) {
    digitalWrite(motorL_IN1, LOW);
    digitalWrite(motorL_IN2, HIGH);
    ledcWriteChannel(0, leftSpeed);
  } else {
    digitalWrite(motorL_IN1, HIGH);
    digitalWrite(motorL_IN2, LOW);
    ledcWriteChannel(0, -leftSpeed);
  }

  // =======================
  // 右馬達
  // =======================
  if (rightSpeed >= 0) {
    digitalWrite(motorR_IN1, LOW);
    digitalWrite(motorR_IN2, HIGH);
    ledcWriteChannel(1, rightSpeed);
  } else {
    digitalWrite(motorR_IN1, HIGH);
    digitalWrite(motorR_IN2, LOW);
    ledcWriteChannel(1, -rightSpeed);
  }
}