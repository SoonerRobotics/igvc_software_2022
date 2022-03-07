#include "common.h"
#include "motor.h"
#include <FlexCAN_T4.h>
IntervalTimer mainTaskTimer;

/*

*/
//

const int ledPin = LED_BUILTIN;
const int encoderLeftB = 6;
const int encoderLeftA = 5;
const int encoderRightA = 8;
const int encoderRightB = 9; 

const int leftMotorPin = 2;
const int rightMotorPin = 3;

motor leftMotor(leftMotorPin, true);
motor rightMotor(rightMotorPin, false);

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
CAN_message_t lastMSG;
float targetLeftSpeed;
float targetRightSpeed;
long counter = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
  pinMode(encoderLeftA, INPUT_PULLUP);
  pinMode(encoderLeftB, INPUT_PULLUP);
  pinMode(encoderRightA, INPUT_PULLUP);
  pinMode(encoderRightB, INPUT_PULLUP);

  Serial.begin(9600);
  mainTaskTimer.begin(set_ms_flags, 1000); // calls this function every 1ms

  attachInterrupt(encoderLeftA, updateLeft, CHANGE);
  //attachInterrupt(encoderLeftB, updateLeft, CHANGE);

  attachInterrupt(encoderRightA, updateRight, CHANGE);
  //attachInterrupt(encoderRightB, updateRight, CHANGE);  

  Can0.begin();
  Can0.setBaudRate(1000000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(receiveMSG);
  Can0.mailboxStatus();

  leftMotor = 0;
  rightMotor = 0;
  digitalWrite(20, HIGH);

  while(Serial.available() == 0) {
    }
}

bool tog = false;

void receiveMSG(const CAN_message_t &msg){
  if (!robotStatus.eStop){
    lastMSG = msg;
    switch (msg.id){
      case 0:
        robotStatus.eStop = 1;
        robotStatus.mStop = 1;
        robotStatus.eStop = 0;
        break;
      case 1: 
        robotStatus.mStop = 1;
        robotStatus.mStart =0;
        break;
      case 10:
        leftMotor = msg.buf[0] * msg.buf[2] / 10.0 / 128.0;
        rightMotor = msg.buf[1] * msg.buf[2] / 10.0 / 128.0;
        break;
    }
  }
}

void updateLeft()
{
  //Serial.printf("left update \n");
  leftMotor.pulse(digitalRead(encoderLeftA),digitalRead(encoderLeftB));
}
void updateRight()
{
  rightMotor.pulse(digitalRead(encoderRightA),digitalRead(encoderRightB));
}

void loop() {
  // put your main code here, to run repeatedly:
  if(robotStatus.eStop){
    leftMotor = 0.0f;
    rightMotor = 0.0f;

    leftMotor.brake();
    rightMotor.brake();

    mainTaskTimer.end();
    Can0.disableFIFO();
    Can0.disableFIFOInterrupt();
    while(true);
  }

  if(!robotStatus.eStop){
    if(mainTasks.task_1ms)
    {


      mainTasks.task_1ms = 0;
    }
    if(mainTasks.task_5ms)
    {
      //Code here
      


      mainTasks.task_5ms = 0;
    }
    if(mainTasks.task_10ms)
    {
      //Code here
      leftMotor.update();
      rightMotor.update();


      Serial.printf("rightspeed %f ", leftMotor.getSpeedEstimate());
      Serial.printf("targetspeed %f \n", targetRightSpeed);
      mainTasks.task_10ms = 0;
    }
    if(mainTasks.task_100ms)
    {
      mainTasks.task_100ms = 0;
      targetRightSpeed = cos(.1 * counter);
      rightMotor = targetRightSpeed;
      leftMotor = targetRightSpeed;
      counter++;
      //Code here
      
      
    }
    if(mainTasks.task_1000ms)
    {

      //Code here
      //leftMotor = 0.15f;
      
    
      mainTasks.task_1000ms = 0;
    }
  }
}
