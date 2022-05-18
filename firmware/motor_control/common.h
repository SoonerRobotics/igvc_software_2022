/*
 * SOONER COMPETITIVE ROBOTICS 
 * Tyler Julian
 * 
 */


#ifndef COMMON_H
#define COMMON_H


typedef  struct{
      short xn;
      short yn;
      short on;
}distance;

distance motorDistances;

typedef struct{
    unsigned int task_1ms : 1;
    unsigned int task_5ms : 1;
    unsigned int task_10ms : 1;
    unsigned int task_100ms : 1;
    unsigned int task_1000ms: 1;
    unsigned int task_50ms: 1;
  
}tasks_t;

typedef struct{
    unsigned int eStop : 1;
    unsigned int mStop : 1;
    unsigned int mStart : 1;
}robotStatus_t;

tasks_t mainTasks;

robotStatus_t robotStatus;

unsigned int ms_count = 0; // count associated with 
void set_ms_flags(){
  ms_count++;
  if (ms_count % 1 == 0)
  {  
    mainTasks.task_1ms = 1;
  }
  if (ms_count % 5 == 0)
  {
    mainTasks.task_5ms = 1;
  }
  if (ms_count % 10 == 0)
  {
    mainTasks.task_10ms = 1;
  }
  if (ms_count % 50 == 0)
  {
    mainTasks.task_50ms = 1; 
  }
  if (ms_count % 100 == 0)
  {
    mainTasks.task_100ms = 1;
  }
  if (ms_count % 1000 == 0)
  {
    mainTasks.task_1000ms = 1;
    ms_count = 0;  
  }
}





#endif
