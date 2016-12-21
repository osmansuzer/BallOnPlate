#include <Servo.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <PID_v1.h>
#include "TouchScreen.h"
#include "I2Cdev.h"
#include "MPU6050.h" 
#include "Wire.h"

#define SERVO_START_VAL 90

//TouchScreen input pins, 0->4
#define YP A0
#define XM A1
#define YM 3 
#define XP 4 

#define EPSILON 10
#define BUF_SIZE 32

/* globals */
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
Servo servo1, servo2;


// for game //////////////////////////


//Gyro (MPU6050)
MPU6050 gyro;

//Gyro outputs
int16_t ax, ay, az;
int16_t gx, gy, gz;



//pins of leds
int leds[4][6];
#define BORDERLINE_PIN 29
#define FIRST_PIN 30 


//Touch Screen
#define TS_MIN_X 160
#define TS_MAX_X 910
 
#define TS_MIN_Y 190
#define TS_MAX_Y 880

//for game1
int game1_x[9];
int game1_y[9];
int step_count=0;
int last_index_x=1;
int last_index_y=1;

int level = 1;
boolean first = true;

int ledXCoordinates[8];
int ledYCoordinates[6];


void setup(void) {
  
  Serial.begin(9600);

  servo1.attach(5);
  servo2.attach(6);

  servo1.write(SERVO_START_VAL);
  servo2.write(SERVO_START_VAL);

  ledXCoordinates[0] = 160;
  ledXCoordinates[1] = 260;
  ledXCoordinates[2] = 365;
  ledXCoordinates[3] = 470;
  ledXCoordinates[4] = 570;
  ledXCoordinates[5] = 675;

  ledYCoordinates[0] = 115;
  ledYCoordinates[1] = 230;
  ledYCoordinates[2] = 340;
  ledYCoordinates[3] = 440;

    //gyro
  Wire.begin();
  gyro.initialize();

  //leds
  int i=0, j=0, pin=FIRST_PIN;
  
  for(;i<4;++i)
    for(j=0;j<6;++j){
      leds[i][j] = pin++;
      pinMode(leds[i][j], OUTPUT);
    }

  pinMode(BORDERLINE_PIN, OUTPUT);

}

void loop(void) {
  if(level == 1 && first == true){
    first = false;
    step_count = 0;
    
    game1_x[0] = 0; game1_y[0] = 0;
    game1_x[1] = 1; game1_y[1] = 0;
    game1_x[2] = 2; game1_y[2] = 0;
    game1_x[3] = 3; game1_y[3] = 0;
    game1_x[4] = 4; game1_y[4] = 0;
    game1_x[5] = 5; game1_y[5] = 0;
    game1_x[6] = 5; game1_y[6] = 1;
    game1_x[7] = 5; game1_y[7] = 2;
    game1_x[8] = 5; game1_y[8] = 3;
    
    int a;
  
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], HIGH);
      delay(250);
    }
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], LOW);
      delay(250);
    }

    delay(250);

    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], HIGH);
      delay(150);
    }
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], LOW);
      delay(150);
    }

    delay(250);
    
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], HIGH);
      delay(50);
    }
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], LOW);
      delay(50);
    }
  
    digitalWrite(29, HIGH); 
  }

  if(level == 2 && first == true){

    first = false;
    step_count = 0;
    
    game1_x[0] = 0; game1_y[0] = 0;
    game1_x[1] = 0; game1_y[1] = 1;
    game1_x[2] = 0; game1_y[2] = 2;
    game1_x[3] = 1; game1_y[3] = 2;
    game1_x[4] = 1; game1_y[4] = 3;
    game1_x[5] = 2; game1_y[5] = 3;
    game1_x[6] = 3; game1_y[6] = 3;
    game1_x[7] = 4; game1_y[7] = 3;
    game1_x[8] = 5; game1_y[8] = 3;
    
    int a;
    
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], HIGH);
      delay(250);
    }
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], LOW);
      delay(250);
    }
    
    delay(250);
    
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], HIGH);
      delay(150);
    }
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], LOW);
      delay(150);
    }
    
    delay(250);
    
    
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], HIGH);
      delay(50);
    }
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], LOW);
      delay(50);
    }
    
    digitalWrite(29, HIGH);
      
  }

  if(level == 3 && first == true){

    first = false;
    step_count = 0;
    
    game1_x[0] = 0; game1_y[0] = 0;
    game1_x[1] = 1; game1_y[1] = 0;
    game1_x[2] = 2; game1_y[2] = 0;
    game1_x[3] = 2; game1_y[3] = 1;
    game1_x[4] = 2; game1_y[4] = 2;
    game1_x[5] = 3; game1_y[5] = 2;
    game1_x[6] = 4; game1_y[6] = 2;
    game1_x[7] = 5; game1_y[7] = 2;
    game1_x[8] = 5; game1_y[8] = 3;
    
    int a;
  
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], HIGH);
      delay(250);
    }
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], LOW);
      delay(250);
    }

    delay(250);

    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], HIGH);
      delay(150);
    }
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], LOW);
      delay(150);
    }

    delay(250);
    
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], HIGH);
      delay(50);
    }
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], LOW);
      delay(50);
    }
  
    digitalWrite(29, HIGH);
      
  }
  
  if(level == 4 && first == true){

    first = false;
    step_count = 0;
    
    game1_x[0] = 0;game1_y[0] = 0;
    game1_x[1] = 1;game1_y[1] = 0;
    game1_x[2] = 1;game1_y[2] = 1;
    game1_x[3] = 2;game1_y[3] = 1;
    game1_x[4] = 2;game1_y[4] = 2;
    game1_x[5] = 2;game1_y[5] = 3;
    game1_x[6] = 3;game1_y[6] = 3;
    game1_x[7] = 4;game1_y[7] = 3;
    game1_x[8] = 5;game1_y[8] = 3;

    int a;

    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], HIGH);
      delay(250);
    }
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], LOW);
      delay(250);
    }

    delay(250);
    
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], HIGH);
      delay(150);
    }
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], LOW);
      delay(150);
    }

    delay(250);
    
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], HIGH);
      delay(50);
    }
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], LOW);
      delay(50);
    }
  
    digitalWrite(29, HIGH);
      
  }

  
   if(level == 5 && first == true){

    first = false;
    step_count = 0;
    
    game1_x[0] = 0;game1_y[0] = 0;
    game1_x[1] = 1;game1_y[1] = 0;
    game1_x[2] = 1;game1_y[2] = 1;
    game1_x[3] = 2;game1_y[3] = 1;
    game1_x[4] = 2;game1_y[4] = 2;
    game1_x[5] = 3;game1_y[5] = 2;
    game1_x[6] = 4;game1_y[6] = 2;
    game1_x[7] = 4;game1_y[7] = 3;
    game1_x[8] = 5;game1_y[8] = 3;

    int a;

    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], HIGH);
      delay(250);
    }
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], LOW);
      delay(250);
    }

    delay(250);
    
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], HIGH);
      delay(150);
    }
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], LOW);
      delay(150);
    }

    delay(250);
    
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], HIGH);
      delay(50);
    }
    for(a=0; a<9; ++a){
      digitalWrite(leds[game1_y[a]][game1_x[a]], LOW);
      delay(50);
    }
  
    digitalWrite(29, HIGH);      
  }
  
  int i;

  gyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  ax = map(ax, -18000, 18000, 0, 180);
  ay = map(ay, -18000, 18000, 0, 180);


  servo1.write(ax);
  servo2.write(ay);
  
  TSPoint p = ts.getPoint();

  if (p.x != 0 && p.y != 1023) {
    int  Xm = map(p.x, 160, 910, 0, 800);
    int Ym = map(p.y, 190, 880, 0, 600);

    int indexX = Xm/100 - 1;
    int indexY = Ym/100 - 1;

    if(step_count != 9 &&
      Xm < ledXCoordinates[game1_x[step_count]] + 100 && Xm > ledXCoordinates[game1_x[step_count]] - 40 &&
        Ym < ledYCoordinates[game1_y[step_count]] + 100 && Ym > ledYCoordinates[game1_y[step_count]] - 40){
          digitalWrite(leds[game1_y[step_count]][game1_x[step_count]], HIGH);
          last_index_x = game1_x[step_count];
          last_index_y = game1_y[step_count];
          ++step_count;
          
        }
    else if(step_count != 9 &&
        Xm < ledXCoordinates[last_index_x] + 100 && Xm > ledXCoordinates[last_index_x] - 100 &&
        Ym < ledYCoordinates[last_index_y] + 100 && Ym > ledYCoordinates[last_index_y] - 100){
          
        }
     else if(step_count==9){
      //win
      for(i=0; i<9; ++i)
          digitalWrite(leds[game1_y[i]][game1_x[i]], LOW);

      
       TSPoint p = ts.getPoint();
       int win = 6;
       while(win!=0){
         digitalWrite(29, LOW);
         delay(500);
         digitalWrite(29, HIGH);
         delay(500);
         TSPoint p = ts.getPoint();
         --win;
       }
       step_count =0;
       first = true;
       ++level;
    }
    else{
      
       for(i=0; i<step_count; ++i)
          digitalWrite(leds[game1_y[i]][game1_x[i]], LOW);
       
       step_count=0;
       digitalWrite(29, LOW);
       delay(250);
       digitalWrite(29, HIGH);
    }

   // Serial.println(step_count);
  }
}
