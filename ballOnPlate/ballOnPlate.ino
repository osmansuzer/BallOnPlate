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

#define EPSILON 10

inline void setDesiredPosition();
/* globals */
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
Servo servo1, servo2;

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

    //get coordinates
    Input=(p.x);  
    Input1=(p.y);
  
    if(myPID.Compute() != false)
      servo1.write(Output = map(Output, -900, 900, 0, 180));//control
    
    if(myPID1.Compute() != false)
      servo2.write(Output1 = map(Output1, -600, 600, 0, 180));//control

    Serial.print("X: ");
    Serial.print(p.x);
    Serial.print(" Y: ");
    Serial.print(p.y);
    Serial.print("\nServo1 : ");
    Serial.println(Output);
    Serial.print("Servo2 : ");
    Serial.println(Output1);
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
  
}
