#include "Arduino.h"
#include <SoftwareSerial.h>

SoftwareSerial bluetooth(7, 8);

int pwm1 = 3;
int pwm2 = 5;
int pwm3 = 6;
int pwm4 = 9;

void setup()
{
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(pwm4, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  randomSeed(analogRead(1));
  Serial.begin(9600);
  bluetooth.begin(9600);
}

void loop()
{
  if (bluetooth.available())
  {
    Serial.println("bt available");
    long val = bluetooth.parseInt();
    String strVal = "value: ";
    strVal += val;
    Serial.println(strVal);
    bluetooth.println(strVal);

    analogWrite(pwm1, val);
    analogWrite(pwm2, val);
    analogWrite(pwm3, val);
    analogWrite(pwm4, val);
  }

  digitalWrite(LED_BUILTIN, random(100) > 80);
  delay(100);
}