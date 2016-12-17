#include <Servo.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <PID_v1.h>
#include "TouchScreen.h"

#define SERVO_START_VAL 90
#define TIME_SAMPLE 30//ms
//TouchScreen input pins, 0->4
#define YP A0
#define XM A1
#define YM 3 
#define XP 4 

#define EPSILON 10
#define BUF_SIZE 12

typedef enum {game, pid} mode_t;

inline void setDesiredPosition();
/* globals */
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
Servo servo1, servo2;
mode_t mode;

char buf[BUF_SIZE];

//PID const x
float Kp = 1.96;          // 1.96                                             
float Kd = 0.66;          // 0.31   // 0.375
float Ki = 0.135;         //  0.0066                                            

//PID const y
float Kp1 = 0.588;         //0.53                                         
float Kd1 = 0.27;         //0.25
float Ki1 = 0.165;        //0.006

double Setpoint, Input, Output; //for X
double Setpoint1, Input1, Output1; //for Y

//init pid controller
//INIT PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); //x
PID myPID1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT); //y

void setup(){

	Serial.begin(9600);

	servo1.attach(5);
	servo2.attach(6);

	servo1.write(SERVO_START_VAL);
	servo2.write(SERVO_START_VAL);

	//Init PID
	myPID.SetMode(AUTOMATIC);
	myPID.SetOutputLimits(-900, 900);
	myPID.SetSampleTime(TIME_SAMPLE);  

	myPID1.SetMode(AUTOMATIC);
	myPID1.SetOutputLimits(-600, 600);
	myPID1.SetSampleTime(TIME_SAMPLE);  

	Setpoint=550;
	Setpoint1=500;

	mode = pid;
}

int stable = 0;

void loop(){

	if(mode == game){
	//TODO 
  }
   
  setDesiredPosition();
  
  TSPoint p = ts.getPoint(); //get coordinates
  
  if(p.x != 0 && p.y != 1023){ //if ball on plate
     
    if(abs(p.y-(int)Setpoint1) <= EPSILON && abs(p.x-(int)Setpoint) <= EPSILON)
  
        ++stable; 
  
    else

      stable = 0;

    if(stable >= 125)

      return ;

    //get coordinates
    Input=(p.x);  
    Input1=(p.y);
  
    if(myPID.Compute() != false)
      servo1.write(Output = map(Output, -900, 900, 0, 180));//control
    
    if(myPID1.Compute() != false)
      servo2.write(Output1 = map(Output1, -600, 600, 0, 180));//control

     memcpy(buf, &p.x, 2);
     memcpy(buf+2, &p.y, 2);
     memcpy(buf+4, &Output, 4);
     memcpy(buf+8, &Output1, 4);
    
     Serial.write(buf, BUF_SIZE);
    
  }else{    
    //restore starting position
    if(servo1.read() != SERVO_START_VAL)
      
      servo1.write(SERVO_START_VAL);
  
   if(servo2.read() != SERVO_START_VAL)
      
      servo2.write(SERVO_START_VAL);      
  }
}

void setDesiredPosition(){

	if(!Serial.available())
		return ;

	int n=0, incoming_size=0;

	while(n!=1)

		n+=Serial.readBytes(buf, 1);

	switch(*buf){

		case 0:
			incoming_size = 9;
			break;
		case 1:
			incoming_size = 25;
			break;
	}

	while(n!=incoming_size)

		n+=Serial.readBytes(buf+n, 1);

  switch(*buf){

    case 0:
      memcpy(&Setpoint, buf+1, 4);
      memcpy(&Setpoint1, buf+5, 4);
      break;
    case 1:
      memcpy(&Kp, buf+1, 4);
      memcpy(&Ki, buf+5, 4);
      memcpy(&Kd, buf+9, 4);
      memcpy(&Kp1, buf+13, 4);
      memcpy(&Ki1, buf+17, 4);
      memcpy(&Kd1, buf+21, 4);
      
      break;
  }

}
