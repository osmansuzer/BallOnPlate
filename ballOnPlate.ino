#include <Servo.h>
#include <stdint.h>
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
//mid coordinates
#define HOME_X 500
#define HOME_Y 500

#define EPSILON 10


inline void setDesiredPosition();
/* globals */
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
Servo servo1, servo2;

//PID const x
/*
float Kp = 1.96;          // 1.96                                             
float Kd = 0.370;          // 0.31   // 0.375
float Ki = 0.007;         //  0.0066                                            
*/


float Kp = 1.95;          // 1.96                                             
float Kd = 0.60;          // 0.31   // 0.375
float Ki = 0.13;         //  0.0066  


//PID const y
float Kp1 = 0.567;         //0.53       //592                                  
float Kd1 = 0.25;         //0.25        //25
float Ki1 = 0.13;        //0.006       //006

double Setpoint, Input, Output; //for X
double Setpoint1, Input1, Output1; //for Y

//TODO
float convertX = 1; // 240.0 / 792;  // converts raw x values to mm. found through manual calibration
float convertY = 1; //300.0 / 660;   // converts raw y values to mm. found through manual calibration

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
  
  //Zapnutie PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-900, 900);
  myPID.SetSampleTime(TIME_SAMPLE);  
  
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(-650, 650);
  myPID1.SetSampleTime(TIME_SAMPLE);  

  Setpoint=550;// ?
  Setpoint1=500;
  
  delay(1000);
 
}

int stable = 0;

void loop(){
   
  setDesiredPosition();
  
  TSPoint p = ts.getPoint(); //get coordinates
  
  if(p.x != 0 && p.y != 1023){ //if ball on plate
     
    if(abs(p.y-(int)Setpoint1) <= EPSILON && abs(p.x-(int)Setpoint) <= EPSILON)
  
        ++stable; 
  
    else

      stable = 0;

    if(stable >= 125)

      return ;

     /*TODO really need to convert to mm?
     */
    Input=(p.x * convertX);  // read and convert X coordinate
    Input1=(p.y * convertY); // read and convert Y coordinate
      
    if(myPID.Compute() != false)
      servo1.write(Output = map(Output, -900, 900, 0, 180));//control
    
    if(myPID1.Compute() != false)
      servo2.write(Output1 = map(Output1, -650, 650, 0, 180));//control

	Serial.println((int)(Setpoint-Input));
	Serial.println((int)(Setpoint1-Input1));
      
    Serial.print("X: ");
    //Serial.print(p.x);
    Serial.print(" Y: ");
    //Serial.print(p.y);
    Serial.print("aServo1 : ");
    Serial.print(Output);
    Serial.print("Servo2 : ");
    Serial.print(Output1);
    Serial.println("------");

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

      Serial.readString();
      
      if(servo1.read() != SERVO_START_VAL)
      
      servo1.write(SERVO_START_VAL);
  
   if(servo2.read() != SERVO_START_VAL)
      
      servo2.write(SERVO_START_VAL);  

  Serial.println("put ball to the desired position");
  delay(3000);
     TSPoint p = ts.getPoint(); //get coordinates

   Setpoint = p.x;
   Setpoint1 = p.y;

	Serial.println("Set ");
	Serial.println(p.x);
	Serial.println("Set1 ");
	Serial.println(p.y);
  
}
