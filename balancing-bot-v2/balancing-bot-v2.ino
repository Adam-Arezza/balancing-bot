#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>
#include "math.h"
#include <PID_v1.h>


FaBo9Axis imu;
float ax,ay,az;
float gx,gy,gz;
float mx,my,mz;
float temp;
float gyroAngle = 0;
int16_t gyroY, gyroRate;
unsigned long currentTime, prevTime = 0, loopTime;
double angle = 0;
double accAngle = 0;
byte rmA = 2;
byte rmB = 3;
byte lmA = 4;
byte lmB = 5;
byte enA = 6;
byte enB = 9;
byte trig = 7;
byte echo = 8;
// double kp = 4.0;
// double ki = 0.5;
// double kd = 1.0;
double angleError = 0.0;
double angleErrorSum = 0.0;
double angleErrorDiff = 0.0;
double prevAngleError = 0.0;
double setPoint = 1.0;
double output = 0;
bool isInitialized = false;
long duration;
int distance;

// PID pid(&angle, &output, &setPoint, 60,5.0,0.9, DIRECT);
PID pid(&angle, &output, &setPoint, 100,150,1.0, DIRECT);

int distance_measured() {
  int distance;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}

void setup() {
  // put your setup code here, to run once:
  imu.begin();
  Serial.begin(115200);
  pinMode(rmA, OUTPUT);
  pinMode(rmB, OUTPUT);
  pinMode(lmA, OUTPUT);
  pinMode(lmB, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  digitalWrite(trig, LOW);
  digitalWrite(echo, LOW);
  digitalWrite(rmA, LOW);
  digitalWrite(rmB, LOW);
  digitalWrite(lmA, LOW);
  digitalWrite(lmB, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(8);
  pid.SetOutputLimits(-255,255);
}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis();
  loopTime = currentTime - prevTime;
  prevTime = currentTime;
  imu.readAccelXYZ(&ax, &ay, &az);
  imu.readGyroXYZ(&gx, &gy, &gz);
  imu.readMagnetXYZ(&mx, &my, &mz);
  imu.readTemperature(&temp);
  ax = -ax;
  gy = gy + -1.85;
  gyroAngle = gyroAngle + gy * loopTime / 1000;
  accAngle = atan2(ax, az) * RAD_TO_DEG;
    if (!isInitialized || gyroAngle < 0.25 && gyroAngle > -0.25) {
    angle = accAngle;
    isInitialized = true;
  }
  angle = 0.75 * gyroAngle + 0.25 * accAngle;
  pid.Compute();
  if(output < 0) {
    int b_out = int(output * 0.9);
    digitalWrite(rmA, LOW);
    digitalWrite(rmB, HIGH);
    digitalWrite(lmA, LOW);
    digitalWrite(lmB, HIGH);
    analogWrite(enA, -output);
    analogWrite(enB, -output);
  }
  else{
    int b_out = int(output * 0.9);
    digitalWrite(rmA, HIGH);
    digitalWrite(rmB, LOW);
    digitalWrite(lmA, HIGH);
    digitalWrite(lmB, LOW);
    analogWrite(enA, output);
    analogWrite(enB, output);
  }
}
