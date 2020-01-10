#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Servo.h"
#include "SPI.h"
#include "RF24.h"
#include "PID_v1.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;
Servo m1, m2, m3, m4;

RF24 radio (7, 8); //RF24 radio(CE_PIN, CSN_PIN);
const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe



int joystick[4];
int x1, y1, x2, y2;
double Input1;
double Setpoint1;
double Input2;
double Setpoint2;
double Input3;
double Setpoint3;
double Output1;
double Output2;
double Output3;

double mo1, mo2, mo3, mo4;


PID myPID1(&Input1, &Output1, &Setpoint1, 1, 0, 0, DIRECT);    //Pitch
PID myPID2(&Input2, &Output2, &Setpoint2, 1, 0, 0, DIRECT);    //roll
PID myPID3(&Input3, &Output3, &Setpoint3, 1, 0, 0, DIRECT);    //yaw

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double yawer, yawrate, lastyaw, t, t1, t2;
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}
void setup()
{
  Serial.begin(9600);


  radio.begin();
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  ;
  m1.attach(3);
  m2.attach(5);
  m3.attach(6);
  m4.attach(9);

  m1.writeMicroseconds(800);
  m2.writeMicroseconds(800);
  m3.writeMicroseconds(800);
  m4.writeMicroseconds(800);

  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);      //turn the PID on
  myPID3.SetMode(AUTOMATIC);      //turn the PID on

  myPID1.SetOutputLimits(-40, 40);
  myPID2.SetOutputLimits(-40, 40);
  myPID3.SetOutputLimits(-20, 20);


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  if (devStatus == 0)
  {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  //delay(15000);
}

void loop()
{
  delay(50);
  if ( radio.available() )
  {
    bool done = false;
    while (!done)
    {
      done = radio.read( joystick, sizeof(joystick) );
      x1 = joystick[0];
      y1 = joystick[1];
      x2 = joystick[2];  //pitch
      y2 = joystick[3];  //roll
    }
  }
  /*Serial.println(x1);
  Serial.println(y1);
  Serial.println(x2);
  Serial.println(y2);*/

  Setpoint3 = y1;
  Setpoint2 = y2;
  Setpoint1 = -x2;

  if (Setpoint1 < 5 && Setpoint1 > -5)
  {
    Setpoint1 = 0;
  }
  if (Setpoint2 < 5 && Setpoint2 > -5)
  {
    Setpoint2 = 0;
  }
  if (Setpoint3 < 5 && Setpoint3 > -5)
  {
    Setpoint3 = 0;
  }

  getangles();


  t2 = millis();
  t = t2 - t1;
  t1 = t2;


  yawer = ypr[0] * 180 / M_PI;
  yawrate = (yawer - lastyaw) * 1000 / t;
  lastyaw = yawer;


  Input1 = (ypr[1] * 180 / M_PI);
  Input2 = (ypr[2] * 180 / M_PI);
  Input3 = yawrate;


  if (Input1 > -1 && Input1 < 1)
  {
    Input1 = 0;
  }
  if (Input2 > -1 && Input2 < 1)
  {
    Input2 = 0;
  }
  
  /*Serial.println(Input1);
  Serial.println(Input2);
  Serial.println(Input3);*/



  myPID1.Compute();
  myPID2.Compute();
  myPID3.Compute();
  
  
  Serial.println(Output1);
  Serial.println(Output2);
  mo1 = constrain((x1 + Output2 + Output1 - Output3), 800, 1500);
  mo2 = constrain((x1 - Output2 + Output1 + Output3), 800, 1500);
  mo3 = constrain((x1 - Output2 - Output1 - Output3), 800, 1500);
  mo4 = constrain((x1 + Output2 - Output1 + Output3), 800, 1500);

  m1.writeMicroseconds(mo1);      //cc
  m2.writeMicroseconds(mo2);      //c
  m3.writeMicroseconds(mo3);      //cc
  m4.writeMicroseconds(mo4);      //c
  /*Serial.println("1  "); Serial.println(mo1);
  Serial.println("2  "); Serial.println(mo2);
  Serial.println("3  "); Serial.println(mo3);
  Serial.println("4  "); Serial.println(mo4);
  Serial.println("------------------------------------------------------------------------------------------------------------------------");
*/
}


void getangles()
{

  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize)
  {
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
  }
  else if (mpuIntStatus & 0x02)
  {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    /*Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);  */
  }




}