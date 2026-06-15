// =======================
// RemoteXY BLE
// =======================

#define REMOTEXY_MODE__ESP32CORE_BLE

#include <BLEDevice.h>
#define REMOTEXY_BLUETOOTH_NAME "RemoteXY"

#include <RemoteXY.h>
#include <math.h>

// =======================
// RemoteXY GUI
// =======================

#pragma pack(push, 1)

uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] = {
  255,1,0,0,0,30,0,19,0,0,0,0,31,1,106,200,1,1,1,0,
  2,31,10,44,22,0,2,26,31,31,79,78,0,79,70,70,0
};

struct {

  uint8_t switch_01;
  uint8_t connect_flag;

} RemoteXY;

#pragma pack(pop)

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
// LED
// =======================

const int greenLed = 16;
const int redLed   = 17;
const int blueLed  = 21;

// =======================
// PID
// =======================

float Kp = 12;
float Kd = 3.5;

float error = 0;
float last_error = 0;
float derivative = 0;

// =======================
// 自動中心
// =======================

float autoCenter = 0;

// =======================
// 濾波
// =======================

float filt[5] = {0,0,0,0,0};
float alpha = 0.65;

// =======================
// 馬達補償
// =======================

int leftMotorTrim = 0;
int rightMotorTrim = 6;

// =======================
// 速度
// =======================

int maxSpeed = 180;

// =======================
// 系統開關
// =======================

bool systemEnabled = false;

// =======================
// LED 狀態
// =======================

void setStatusLED(int state)
{
  digitalWrite(greenLed, LOW);
  digitalWrite(redLed, LOW);
  digitalWrite(blueLed, LOW);

  switch (state)
  {
    case 0:
      digitalWrite(greenLed, HIGH);
      break;

    case 1:
      digitalWrite(redLed, HIGH);
      break;

    case 2:
      digitalWrite(blueLed, HIGH);
      break;
  }
}

// =======================
// 停止馬達
// =======================

void motorStop()
{
  ledcWriteChannel(0, 0);
  ledcWriteChannel(1, 0);

  digitalWrite(motorL_IN1, LOW);
  digitalWrite(motorL_IN2, LOW);

  digitalWrite(motorR_IN1, LOW);
  digitalWrite(motorR_IN2, LOW);
}

// =======================
// 失線搜尋
// =======================

void searchLine()
{
  if (last_error > 0)
  {
    digitalWrite(motorL_IN1, LOW);
    digitalWrite(motorL_IN2, HIGH);

    digitalWrite(motorR_IN1, HIGH);
    digitalWrite(motorR_IN2, LOW);
  }
  else
  {
    digitalWrite(motorL_IN1, HIGH);
    digitalWrite(motorL_IN2, LOW);

    digitalWrite(motorR_IN1, LOW);
    digitalWrite(motorR_IN2, HIGH);
  }

  ledcWriteChannel(0, 130);
  ledcWriteChannel(1, 130);
}

// =======================
// Setup
// =======================

void setup()
{
  Serial.begin(115200);

  RemoteXY_Init();

  analogReadResolution(12);

  for (int i = 0; i < 5; i++)
  {
    pinMode(sensorPins[i], INPUT);
  }

  pinMode(greenLed, OUTPUT);
  pinMode(redLed, OUTPUT);
  pinMode(blueLed, OUTPUT);

  pinMode(motorL_IN1, OUTPUT);
  pinMode(motorL_IN2, OUTPUT);

  pinMode(motorR_IN1, OUTPUT);
  pinMode(motorR_IN2, OUTPUT);

  ledcAttachChannel(motorL_PWM, 1000, 8, 0);
  ledcAttachChannel(motorR_PWM, 1000, 8, 1);

  motorStop();
}

// =======================
// Loop
// =======================

void loop()
{
  RemoteXY_Handler();

  // -----------------------
  // BLE未連線
  // -----------------------

  if (!RemoteXY.connect_flag)
  {
    motorStop();
    setStatusLED(2);
    return;
  }

  systemEnabled = RemoteXY.switch_01;

  // -----------------------
  // 待命模式
  // -----------------------

  if (!systemEnabled)
  {
    motorStop();
    setStatusLED(0);

    error = 0;
    last_error = 0;
    derivative = 0;

    autoCenter = 0;

    for (int i = 0; i < 5; i++)
      filt[i] = 0;

    return;
  }

  // -----------------------
  // 尋線模式
  // -----------------------

  setStatusLED(1);

  int raw[5];

  for (int i = 0; i < 5; i++)
  {
    raw[i] = analogRead(sensorPins[i]);
  }

  float sum = 0;
  float position = 0;

  for (int i = 0; i < 5; i++)
  {
    int v = 4095 - raw[i];

    filt[i] =
      alpha * filt[i] +
      (1.0 - alpha) * v;

    sum += filt[i];
    position += filt[i] * w[i];
  }

  // -----------------------
  // 失線
  // -----------------------

  if (sum < 50)
  {
    searchLine();
    return;
  }

  position /= sum;

  // -----------------------
  // 自動中心校正
  // -----------------------

  autoCenter =
    autoCenter * 0.995 +
    position * 0.005;

  error = position - autoCenter;

  if (abs(error) < 0.04)
    error = 0;

  // -----------------------
  // PID
  // -----------------------

  derivative = error - last_error;

  float turn =
      Kp * error +
      Kd * derivative;

  last_error = error;

  // -----------------------
  // 非線性壓縮
  // -----------------------

  turn = tanh(turn * 0.03) * 120;

  // -----------------------
  // 動態速度
  // -----------------------

  int speedAdjust = abs(error) * 8;

  int baseSpeed =
      constrain(maxSpeed - speedAdjust,
                110,
                maxSpeed);

  float turnScale = 0.5;

  int leftSpeed =
      baseSpeed - turn * turnScale;

  int rightSpeed =
      baseSpeed + turn * turnScale;

  leftSpeed += leftMotorTrim;
  rightSpeed += rightMotorTrim;

  leftSpeed =
      constrain(leftSpeed, -255, 255);

  rightSpeed =
      constrain(rightSpeed, -255, 255);

  // -----------------------
  // 差速限制
  // -----------------------

  int diff = rightSpeed - leftSpeed;
  int maxDiff = 55;

  if (diff > maxDiff)
    rightSpeed = leftSpeed + maxDiff;

  if (diff < -maxDiff)
    leftSpeed = rightSpeed + maxDiff;

  // -----------------------
  // 左馬達
  // -----------------------

  if (leftSpeed >= 0)
  {
    digitalWrite(motorL_IN1, LOW);
    digitalWrite(motorL_IN2, HIGH);

    ledcWriteChannel(0, leftSpeed);
  }
  else
  {
    digitalWrite(motorL_IN1, HIGH);
    digitalWrite(motorL_IN2, LOW);

    ledcWriteChannel(0, -leftSpeed);
  }

  // -----------------------
  // 右馬達
  // -----------------------

  if (rightSpeed >= 0)
  {
    digitalWrite(motorR_IN1, LOW);
    digitalWrite(motorR_IN2, HIGH);

    ledcWriteChannel(1, rightSpeed);
  }
  else
  {
    digitalWrite(motorR_IN1, HIGH);
    digitalWrite(motorR_IN2, LOW);

    ledcWriteChannel(1, -rightSpeed);
  }
}
