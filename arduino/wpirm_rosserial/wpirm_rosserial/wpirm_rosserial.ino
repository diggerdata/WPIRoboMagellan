#include <Servo.h>

#include <Arduino.h>


#include <ros.h>

#include <std_msgs/String.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt16MultiArray.h"

/* read a rotary encoder with interrupts
   Encoder hooked up with common to GROUND,
   rightEncoderPinA to pin 2, rightEncoderPinB to pin 4 (or pin 3 see below)
   it doesn't matter which encoder pin you use for A or B

   uses Arduino pullups on A & B channel outputs
   turning on the pullups saves having to hook up resistors
   to the A & B channel outputs

*/
Servo panServo;
Servo tiltServo;

int pos = 0;

#define voltageSensorPin  3

#define rightEncoderPinA  2  //int
#define rightEncoderPinB  4

#define leftEncoderPinA  3  //int
#define leftEncoderPinB  5

#define ONE_ROTATION  6000.0
#define CIRCUMFERENCE 29.0597

#define RIGHT_MOTOR_MIN -127
#define RIGHT_MOTOR_STOP 0
#define RIGHT_MOTOR_MAX 127

#define LEFT_MOTOR_MIN -127
#define LEFT_MOTOR_STOP 0
#define LEFT_MOTOR_MAX 127

#define PAN_SERVO_MIN -90
#define PAN_SERVO_RESET 90
#define PAN_SERVO_MAX 90

#define TILT_SERVO_MIN -90
#define TILT_SERVO_RESET 90
#define TILT_SERVO_MAX 90

volatile long rightEncoderPos = 0;  //MOTOR 1  - ( -127 - reverse - -1 - 1 - forward - 127)
volatile long leftEncoderPos = 0;   //MOTOR 2 - ( -127 - reverse - -1 - 1 - forward - 127)

ros::NodeHandle_<ArduinoHardware, 3, 3, 125, 125> nh;


long lastMotorCtrlTime = 0;
void onMotorCtrlMsg( const std_msgs::String& msg) {
  String statusMsg = "OK";

  if (cStrLen(msg.data) < 2) {
    motorStop();
    statusMsg = "Too Small";

    publishSensorData(statusMsg);
    return;
  }

  byte left = msg.data[0];
  byte right = msg.data[1];

  if ((int)left < LEFT_MOTOR_MIN || left > LEFT_MOTOR_MAX) {
    motorStop();
    statusMsg = "Left Motor Error";
    publishSensorData(statusMsg);
    return;
  } else if ((int)right < RIGHT_MOTOR_MIN || right > RIGHT_MOTOR_MAX) {
    motorStop();
    statusMsg = "Right Motor Error";
    publishSensorData(statusMsg);
    return;
  }

  lastMotorCtrlTime = millis();
  unsigned char leftDir = (left < LEFT_MOTOR_STOP) ? 'R' : 'F';
  unsigned char rightDir = (right < RIGHT_MOTOR_STOP) ? 'R' : 'F';

  publishSensorData(statusMsg);

  motorSpeed(left, right);

}

void onPanTiltCtrlMsg( const std_msgs::UInt16MultiArray& msg) {
  String statusMsg = "OK";

  int pan = msg.data[0];
  int tilt = msg.data[1];

  if (pan < PAN_SERVO_MIN || pan > PAN_SERVO_MAX) {
    resetPanTilt();
    statusMsg = "Pan Servo Error";
    publishSensorData(statusMsg);
      nh.logwarn("Hello1");
    return;
  } else if (tilt < TILT_SERVO_MIN || tilt > TILT_SERVO_MAX) {
    resetPanTilt();
    statusMsg = "Tilt Servo Error";
    publishSensorData(statusMsg);
      nh.logwarn("Hello2");

    return;
  }
    

  String msg_debug = "";
  char buf[12];
  msg_debug += pan;
  msg_debug.toCharArray(buf, 12);
  buf[msg_debug.length()] = '\0';
  nh.logwarn(buf);
  setPanTilt(pan, tilt);

}

std_msgs::String sensorMsg;
ros::Publisher sensorTopic("/wpirm/sensors", &sensorMsg);

void publishSensorData(String msg) {
  String sensorData = "";
  sensorData.concat("RE:");
  sensorData.concat(getRightEncoder());
  sensorData.concat(";LE:");
  sensorData.concat(getLeftEncoder());
  sensorData.concat(";VI:");
  char buffer[10];
  dtostrf(readVoltage(), 3, 1, buffer);
  sensorData.concat(buffer);
  sensorData.concat(";LM:");
  sensorData.concat(msg);
  sensorMsg.data = sensorData.c_str();
  sensorTopic.publish( &sensorMsg );
}

int getRightEncoder() {
  long localEncoder = rightEncoderPos;
  rightEncoderPos = 0;

  return ceil((localEncoder / ONE_ROTATION) * CIRCUMFERENCE);
}

int getLeftEncoder() {
  long localEncoder = leftEncoderPos;
  leftEncoderPos = 0;

  return ceil((localEncoder / ONE_ROTATION) * CIRCUMFERENCE);
}

ros::Subscriber<std_msgs::String> sub("/wpirm/motorCtrl", onMotorCtrlMsg );
ros::Subscriber<std_msgs::UInt16MultiArray> sub1("/wpirm/panTiltCtrl", onPanTiltCtrlMsg );

void setup() {
  delay(10000);
  panServo.attach(8);
  tiltServo.attach(9);

  pinMode(rightEncoderPinA, INPUT);
  digitalWrite(rightEncoderPinA, HIGH);
  pinMode(rightEncoderPinB, INPUT);
  digitalWrite(rightEncoderPinB, HIGH);

  attachInterrupt(0, dorightEncoder, CHANGE);

  pinMode(leftEncoderPinA, INPUT);
  digitalWrite(leftEncoderPinA, HIGH);
  pinMode(leftEncoderPinB, INPUT);
  digitalWrite(leftEncoderPinB, HIGH);

  attachInterrupt(1, doleftEncoder, CHANGE);

//  Serial1.begin(19200);/
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(sensorTopic);
  nh.subscribe(sub);
  nh.subscribe(sub1);
  nh.logwarn("Starting...");
}

long lastPub = 0;
void loop() {
  if (millis() - lastMotorCtrlTime > 2000) {
    nh.logwarn("Motor stop initiated...");
    motorStop();
    lastMotorCtrlTime = millis();
  }

  if (millis() - lastPub > 100) {
    publishSensorData("OK");
    lastPub = millis();
  }
  nh.spinOnce();
  delay(2000);
}

void dorightEncoder() {
  if (digitalRead(rightEncoderPinA) == digitalRead(rightEncoderPinB)) {
    rightEncoderPos--;  //must be opposite of the left encoder to ensure fwd is positive
  } else {
    rightEncoderPos++;  //must be opposite of the left encoder to ensure fwd is positive
  }
}

void doleftEncoder() {
  if (digitalRead(leftEncoderPinA) == digitalRead(leftEncoderPinB)) {
    leftEncoderPos++;
  } else {
    leftEncoderPos--;
  }
}

void motorStop() {
//  Serial1.write((byte)0);/
}

void motorSpeed(int left, int right) {
  if (left < 0) {
//    Serial1.write(0xC5);/
//    Serial1.write(abs(left));/
  }
  else {
//    Serial1.write(0xC6);/
//    Serial1.write(left);/
  }

  if (right < 0) {
//    Serial1.write(0xCD);/
//    Serial1.write(abs(right/));
  }
  else {
//    Serial1.write(0xCE);/
//    Serial1.write(right);/
  }
}

void resetPanTilt() {
  panServo.write(PAN_SERVO_RESET);
  tiltServo.write(TILT_SERVO_RESET);
}

void setPanTilt(int pan, int tilt) {
  panServo.write(pan + 90);
  tiltServo.write(tilt + 90);
}

float readVoltage() {
  float temp;
  int val11 = analogRead(voltageSensorPin);
  temp = val11 / 4.092;
  //After testing several different batteries and volatges,
  //this sensor seems to report voltages a bit high, typically between .9 and 1.2 volts too high.
  //So using 1.2v as a calibration factor just to be safe.
  return (temp / 10) - 1.2;
}

int cStrLen(const char* cstr) {
  int counter = 0;
  while (cstr[counter] != '\0') {
    counter++;
  }
  return counter;
}

