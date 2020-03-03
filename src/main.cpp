#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <SoftwareSerial.h>
#include <PID_v1.h>

#define MOTOR_1_PIN 3
#define MOTOR_2_PIN 5
#define MOTOR_3_PIN 6
#define MOTOR_4_PIN 9
#define INTERRUPT_PIN 2

#define PITCH 1
#define ROLL 2
#define YAW 0

#define MOTOR_1_SPD 0
#define MOTOR_2_SPD 1
#define MOTOR_3_SPD 2
#define MOTOR_4_SPD 3

uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
uint8_t speed[3];
uint8_t baseSpeed;

Quaternion q;
VectorFloat gravity;

float ypr[3];

double pitchSet, pitchIn, pitchOut, rollSet, rollIn, rollOut, yawSet, yawIn, yawOut;

PID pitchPID(&pitchIn, &pitchOut, &pitchSet, 0.5, 0.05, 0.05, DIRECT);
PID rollPID(&rollIn, &rollOut, &rollSet, 0.5, 0.05, 0.05, DIRECT);
PID yawPID(&yawIn, &yawOut, &yawSet, 0.5, 0.05, 0.05, DIRECT);

MPU6050 mpu;
SoftwareSerial bluetooth(7, 8);

bool stop = true;
volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
  Wire.begin();
  mpu.initialize();
  bluetooth.begin(9600);

  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(MOTOR_1_PIN, OUTPUT);
  pinMode(MOTOR_2_PIN, OUTPUT);
  pinMode(MOTOR_3_PIN, OUTPUT);
  pinMode(MOTOR_4_PIN, OUTPUT);

  randomSeed(analogRead(1));

  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-30, 30);
  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(-30, 30);
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetOutputLimits(-30, 30);

  mpu.setXAccelOffset(-696);
  mpu.setYAccelOffset(-1670);
  mpu.setZAccelOffset(1312);
  mpu.setXGyroOffset(19);
  mpu.setYGyroOffset(-23);
  mpu.setZGyroOffset(35);
  mpu.setDMPEnabled(true);

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();
}

void processMPUInterruption()
{
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount >= 1024)
  {
    mpu.resetFIFO();
    return;
  }

  if (!(mpuIntStatus & 0x02))
  {
    return;
  }

  while (fifoCount < packetSize)
  {
    fifoCount = mpu.getFIFOCount();
  }

  mpu.getFIFOBytes(fifoBuffer, packetSize);
  fifoCount -= packetSize;

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  pitchIn = ypr[PITCH] * 180 / M_PI;
  rollIn = ypr[ROLL] * 180 / M_PI;
  yawIn = ypr[YAW] * 180 / M_PI;
}

void sendLog()
{
  String jsonStr = "$[[";
  jsonStr += yawIn;
  jsonStr += ",";
  jsonStr += pitchIn;
  jsonStr += ",";
  jsonStr += rollIn;
  jsonStr += "],[";
  jsonStr += yawOut;
  jsonStr += ",";
  jsonStr += pitchOut;
  jsonStr += ",";
  jsonStr += rollOut;
  jsonStr += "],[";
  jsonStr += yawSet;
  jsonStr += ",";
  jsonStr += pitchSet;
  jsonStr += ",";
  jsonStr += rollSet;
  jsonStr += "],[";
  jsonStr += speed[MOTOR_1_SPD];
  jsonStr += ",";
  jsonStr += speed[MOTOR_2_SPD];
  jsonStr += ",";
  jsonStr += speed[MOTOR_3_SPD];
  jsonStr += ",";
  jsonStr += speed[MOTOR_4_SPD];
  jsonStr += "]]";
  bluetooth.println(jsonStr);
}

void readCommand()
{
  int cmd = bluetooth.read();
  if (cmd == 49)
  {
    pitchSet += 1;
  }

  if (cmd == 50)
  {
    rollSet += 1;
  }

  if (cmd == 51)
  {
    pitchSet -= 1;
  }

  if (cmd == 52)
  {
    rollSet -= 1;
  }

  if (cmd == 53)
  {
    baseSpeed = constrain(baseSpeed + 20, 0, 255);
  }

  if (cmd == 54)
  {
    baseSpeed = constrain(baseSpeed - 20, 0, 255);
  }

  if (cmd == 55)
  {
    baseSpeed = 0;
  }

  if (cmd == 56)
  {
    pitchSet = 0;
    rollSet = 0;
  }

  if (cmd == 57)
  {
    stop = !stop;
  }
}

void applySpeed()
{
  if (stop)
  {
    analogWrite(MOTOR_1_PIN, 0);
    analogWrite(MOTOR_2_PIN, 0);
    analogWrite(MOTOR_3_PIN, 0);
    analogWrite(MOTOR_4_PIN, 0);
    return;
  }

  analogWrite(MOTOR_1_PIN, speed[MOTOR_1_SPD]);
  analogWrite(MOTOR_2_PIN, speed[MOTOR_2_SPD]);
  analogWrite(MOTOR_3_PIN, speed[MOTOR_3_SPD]);
  analogWrite(MOTOR_4_PIN, speed[MOTOR_4_SPD]);
}

void mapOutputToSpeed()
{
  float rollShift = rollOut / 2;
  float pitchShift = rollOut / 2;
  float yawShift = yawOut / 2;

  speed[MOTOR_1_SPD] = constrain(baseSpeed + rollShift + yawShift, 0, 255);
  speed[MOTOR_3_SPD] = constrain(baseSpeed - rollShift + yawShift, 0, 255);
  speed[MOTOR_2_SPD] = constrain(baseSpeed + pitchShift - yawShift, 0, 255);
  speed[MOTOR_4_SPD] = constrain(baseSpeed - pitchShift - yawShift, 0, 255);
}

void loop()
{
  if (mpuInterrupt)
  {
    processMPUInterruption();
  }

  if (bluetooth.available())
  {
    readCommand();
  }

  pitchPID.Compute();
  rollPID.Compute();
  yawPID.Compute();

  mapOutputToSpeed();
  applySpeed();
  sendLog();
  digitalWrite(LED_BUILTIN, random(100) > 90);
}
