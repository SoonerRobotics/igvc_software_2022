#include "common.h"
IntervalTimer mainTaskTimer;
const int ledPin = LED_BUILTIN;

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  mainTaskTimer.begin(set_ms_flags, 1000); // calls this function every 1ms
}

void loop() {
  // put your main code here, to run repeatedly:
  if(mainTasks.task_1ms)
  {
    //Code here

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
