#include "common.h"

#include <FlexCAN_T4.h>
IntervalTimer mainTaskTimer;

/*

*/
//

const int ledPin = LED_BUILTIN;
const int light1 = 1;
const int light2 = 2;
const int light3 = 3;
const int light4 = 4;
const int estop_sig = 20;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
CAN_message_t lastMSG;

long counter = 0;
int lightsON = 1;

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
  pinMode(light1, OUTPUT);
  pinMode(light2, OUTPUT);
  pinMode(light3, OUTPUT);
  pinMode(light4, OUTPUT);
  pinMode(estop_sig, INPUT);

  Serial.begin(9600);
  mainTaskTimer.begin(set_ms_flags, 1000); // calls this function every 1ms
 
  Can0.begin();
  Can0.setBaudRate(100000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(receiveMSG);
  Can0.mailboxStatus();

  digitalWrite(light1, LOW);
  digitalWrite(light2, LOW);
  digitalWrite(light3, LOW);
  digitalWrite(light4, LOW);

  robotStatus.eStop = 0;
  robotStatus.mStop = 1;
  robotStatus.mStart = 0;

  //digitalWrite(estop_sig, HIGH);

}

bool tog = false;

void receiveMSG(const CAN_message_t &msg){
    lastMSG = msg;
    tog = !tog;
    digitalWrite(ledPin, tog);
    switch (msg.id){
      case 0:

        lightsON = 1;
        break;
      case 1: 

        lightsON = 1;
        break;
      case 9:

        break;
      case 13:
        lightsON = msg.buf[0];
  }
}

bool tog_audio = false;
void loop() {
  // put your main code here, to run repeatedly:
  if(lightsON == 1){
          digitalWrite(light1, LOW);
          digitalWrite(light2, LOW);
          digitalWrite(light3, LOW);
          digitalWrite(light4, LOW);
  }
    // put your main code here, to run repeatedly:
  if(lightsON == 0){
          digitalWrite(light1, HIGH);
          digitalWrite(light2, HIGH);
          digitalWrite(light3, HIGH);
          digitalWrite(light4, HIGH);
  }
  
  if(lightsON == 2){
      if(mainTasks.task_1ms)
      {
        tog_audio = !tog_audio;
        digitalWrite(light4, tog_audio);
        mainTasks.task_1ms = 0;
      }
      if(mainTasks.task_5ms)
      {
        //Code here
        
  
  
        mainTasks.task_5ms = 0;
      }
      if(mainTasks.task_10ms)
      {
  
  
        //
        //Serial.printf("rightspeed %f ", leftMotor.getSpeedEstimate());
        //Serial.printf("targetspeed %f \n", targetRightSpeed);
        mainTasks.task_10ms = 0;
      }
      if(mainTasks.task_100ms)
      {
        mainTasks.task_100ms = 0;
  
        
        
      }
      if(mainTasks.task_1000ms)
      {
        tog = !tog;
        digitalWrite(light1, tog);
        digitalWrite(light2, tog);
        digitalWrite(light3, tog);
        Serial.println("debug");

  
        mainTasks.task_1000ms = 0;
      
    }
  }
}
