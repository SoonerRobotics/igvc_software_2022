#include "common.h"
#include "motor.h"
IntervalTimer mainTaskTimer;

/*

*/
//

const int ledPin = LED_BUILTIN;
const int encoderLeftA = -1;
const int encoderLeftB = -1;
const int encoderRightA = -1;
const int encoderRightB = -1; 

const int leftMotorPin = -1;
const int rightMotorPin = -1;

motor leftMotor(leftMotorPin, false);
motor rightMotor(rightMotorPin, true);

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
 CAN_message_t lastMSG;
 float targetLeftSpeed;
 float targetRightSpeed;

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
  pinMode(encoderLeftA, INPUT);
  pinMode(encoderLeftB, INPUT);
  pinMode(encoderRightA, INPUT);
  pinMode(encoderRightB, INPUT);

  Serial.begin(9600);
  mainTaskTimer.begin(set_ms_flags, 1000); // calls this function every 1ms

  attachInterrupt(encoderLeftA, updateLeft, RISE);
  attachInterrupt(encoderLeftB, updateLeft, FALL);

  attachInterrupt(encoderRightA, updateRight, RISE);
  attachInterrupt(encoderRightB, updateRight, FALL);  

  Can0.begin();
  Can0.setBaudRate(1000000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(receiveMSG);
  Can0.mailboxStatus();

}

void receiveMSG(const CAN_message_t &msg){
  if (!robotStatus.eStop){
    self.lastMSG = msg;
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
      //Code here
      leftMotor.update();
      rightMotor.update();

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
      
      mainTasks.task_10ms = 0;
    }
    if(mainTasks.task_100ms)
    {
      mainTasks.task_100ms = 0;
      //Code here
      
    }
    if(mainTasks.task_1000ms)
    {
      //Code here
    
      mainTasks.task_1000ms = 0;
    }
  }
}
