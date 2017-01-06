//final

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
#define TIME_SAMPLE 30//ms
#define SEND_PERIOD 150 //ms
//TouchScreen input pins, 0->4
#define YP A0
#define XM A1
#define YM 3
#define XP 4

#define EPSILON 10
#define BUF_SIZE 32

typedef enum {game, pid} mode_t;

inline void setDesiredPosition();
inline void game_loop();

inline void one_high();
inline void one_low();

inline void two_high();
inline void two_low();

inline void tree_high();
inline void tree_low();

inline void four_high();
inline void four_low();

inline void five_high();
inline void five_low();

inline void high_borderline();
inline void low_borderline();

inline void draw_square();
inline void draw_triangle();

inline void send_position(int16_t x, int16_t y, float agile1, float agile2);
inline void send_all(int16_t x, int16_t y, float agile1, float agile2, char**leds);
/* globals */
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
Servo servo1, servo2;
mode_t mode;

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
int over_count=0;

int last_index_x=1;
int last_index_y=1;

int last_pos_x;
int last_pos_y;

char game_leds[6][8];

int level = 1;
boolean first = true;
boolean return_game=false;
boolean again_flag = false;

int ledXCoordinates[8];
int ledYCoordinates[6];

char buf[BUF_SIZE];

//PID const x
float Kp = 1.98;          // 1.96                                             
float Kd = 1.0;          // 0.31   // 0.375
float Ki = 0.067;         //  0.0066                                            

//PID const y
float Kp1 = 1.0;         //0.53                                         
float Kd1 = 0.5;         //0.25
float Ki1 = 0.002;        //0.006

double Setpoint, Input, Output; //for X
double Setpoint1, Input1, Output1; //for Y

long lastSent = 0;

//init pid controller
//INIT PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); //x
PID myPID1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT); //y

void setup(){

    Serial.begin(115200);

    while(!Serial)
        ;

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

    //init leds
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
 
    for(i=0; i<6;++i){
      for(j=0; j<8; ++j)
        game_leds[i][j] ='f';
    }


    pinMode(BORDERLINE_PIN, OUTPUT);
}

int stable = 0;

void loop(){

    if(mode == game){
        game_loop();
        return ;
    }

    setDesiredPosition();

    TSPoint p = ts.getPoint(); //get coordinates

    if(p.x != 0 && p.y != 1023){ //if ball on plate

        if(abs(p.y-(int)Setpoint1) <= EPSILON && abs(p.x-(int)Setpoint) <= EPSILON)

            ++stable;

        else

            stable = 0;

        if(stable >= 125){

            if(return_game)
              return ;
        }
        //get coordinates
        Input=(p.x);
        Input1=(p.y);

        if(myPID.Compute() != false)
            servo1.write(Output = map(Output, -900, 900, 0, 180));//control

        if(myPID1.Compute() != false)
            servo2.write(Output1 = map(Output1, -600, 600, 0, 180));//control

        if(millis()-lastSent >= SEND_PERIOD){

            lastSent=millis();

            Serial.write((char*)&p.x, 2);
            Serial.write((char*)&p.y, 2);
            Serial.write((char*)&Output, 4);
            Serial.write((char*)&Output1, 4);
        }else{
            delay(1);
        }

        delay(29);

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

    while(n < 1)
        n+=Serial.readBytes(buf, 1);

    switch(*buf){

        case '0':
            mode=pid;
            incoming_size = 9;
            break;
        case '1':
            mode = game;
            return ;
     }

    while(n < incoming_size)

        n += Serial.readBytes(buf+n, incoming_size-n);


    switch(*buf){

        case '0':
            memcpy(&Setpoint, buf+1, 4);
            memcpy(&Setpoint1, buf+5, 4);
            break;
    }
}
void game_loop(){

  setDesiredPosition();
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

        int a=3;
        
        while(a>0){
          one_high();
          delay(250);
          one_low();
          delay(250);
          --a;
        }

        
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
        int a=3;
        
        while(a>0){
          two_high();
          delay(250);
          two_low();
          delay(250);
          --a;
        }

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

        int a=3;
        
        while(a>0){
          tree_high();
          delay(250);
          tree_low();
          delay(250);
          --a;
        }

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

        int a=3;
        
        while(a>0){
          four_high();
          delay(250);
          four_low();
          delay(250);
          --a;
        }

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

        int a=3;
        
         while(a>0){
          five_high();
          delay(250);
          five_low();
          delay(250);
          --a;
        }

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
            last_pos_x = p.x;
            last_pos_y = p.y;
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

            
        }
    }
    
}



void one_high(){

  
  
  digitalWrite(leds[1][4], HIGH);
  digitalWrite(leds[1][1], HIGH);
  digitalWrite(leds[2][5], HIGH);
  digitalWrite(leds[2][4], HIGH);
  digitalWrite(leds[2][3], HIGH);
  digitalWrite(leds[2][2], HIGH);
  digitalWrite(leds[2][1], HIGH);
  digitalWrite(leds[3][1], HIGH);

}

void one_low(){

  digitalWrite(leds[1][4], LOW);
  digitalWrite(leds[1][1], LOW);
  digitalWrite(leds[2][5], LOW);
  digitalWrite(leds[2][4], LOW);
  digitalWrite(leds[2][3], LOW);
  digitalWrite(leds[2][2], LOW);
  digitalWrite(leds[2][1], LOW);
  digitalWrite(leds[3][1], LOW);

}

void two_high(){
  
  digitalWrite(leds[1][5], HIGH);
  digitalWrite(leds[1][3], HIGH);
  digitalWrite(leds[1][2], HIGH);
  digitalWrite(leds[1][1], HIGH);
  digitalWrite(leds[2][5], HIGH);
  digitalWrite(leds[2][3], HIGH);
  digitalWrite(leds[2][1], HIGH);
  digitalWrite(leds[3][5], HIGH);
  digitalWrite(leds[3][4], HIGH);
  digitalWrite(leds[3][3], HIGH);
  digitalWrite(leds[3][1], HIGH);


}

void two_low(){
  digitalWrite(leds[1][5], LOW);
  digitalWrite(leds[1][3], LOW);
  digitalWrite(leds[1][2], LOW);
  digitalWrite(leds[1][1], LOW);
  digitalWrite(leds[2][5], LOW);
  digitalWrite(leds[2][3], LOW);
  digitalWrite(leds[2][1], LOW);
  digitalWrite(leds[3][5], LOW);
  digitalWrite(leds[3][4], LOW);
  digitalWrite(leds[3][3], LOW);
  digitalWrite(leds[3][1], LOW);

  
}

void tree_high(){
  digitalWrite(leds[1][5], HIGH);
  digitalWrite(leds[1][3], HIGH);
  digitalWrite(leds[1][1], HIGH);
  digitalWrite(leds[2][5], HIGH);
  digitalWrite(leds[2][3], HIGH);
  digitalWrite(leds[2][1], HIGH);
  digitalWrite(leds[3][5], HIGH);
  digitalWrite(leds[3][4], HIGH);
  digitalWrite(leds[3][3], HIGH);
  digitalWrite(leds[3][2], HIGH);
  digitalWrite(leds[3][1], HIGH);

}

void tree_low(){

  digitalWrite(leds[1][5], LOW);
  digitalWrite(leds[1][3], LOW);
  digitalWrite(leds[1][1], LOW);
  digitalWrite(leds[2][5], LOW);
  digitalWrite(leds[2][3], LOW);
  digitalWrite(leds[2][1], LOW);
  digitalWrite(leds[3][5], LOW);
  digitalWrite(leds[3][4], LOW);
  digitalWrite(leds[3][3], LOW);
  digitalWrite(leds[3][2], LOW);
  digitalWrite(leds[3][1], LOW);

}

void four_high(){
  digitalWrite(leds[1][5], HIGH);
  digitalWrite(leds[1][4], HIGH);
  digitalWrite(leds[1][3], HIGH);
  digitalWrite(leds[2][3], HIGH);
  digitalWrite(leds[3][5], HIGH);
  digitalWrite(leds[3][4], HIGH);
  digitalWrite(leds[3][3], HIGH);
  digitalWrite(leds[3][2], HIGH);
  digitalWrite(leds[3][1], HIGH);


}

void four_low(){
  digitalWrite(leds[1][5], LOW);
  digitalWrite(leds[1][4], LOW);
  digitalWrite(leds[1][3], LOW);
  digitalWrite(leds[2][3], LOW);
  digitalWrite(leds[3][5], LOW);
  digitalWrite(leds[3][4], LOW);
  digitalWrite(leds[3][3], LOW);
  digitalWrite(leds[3][2], LOW);
  digitalWrite(leds[3][1], LOW);


}

void five_high(){
  
  digitalWrite(leds[1][5], HIGH);
  digitalWrite(leds[1][4], HIGH);
  digitalWrite(leds[1][3], HIGH);
  digitalWrite(leds[1][1], HIGH);
  digitalWrite(leds[2][5], HIGH);
  digitalWrite(leds[2][3], HIGH);
  digitalWrite(leds[2][1], HIGH);
  digitalWrite(leds[3][5], HIGH);
  digitalWrite(leds[3][3], HIGH);
  digitalWrite(leds[3][2], HIGH);
  digitalWrite(leds[3][1], HIGH);


}

void five_low(){
  
  digitalWrite(leds[1][5], LOW);
  digitalWrite(leds[1][4], LOW);
  digitalWrite(leds[1][3], LOW);
  digitalWrite(leds[1][1], LOW);
  digitalWrite(leds[2][5], LOW);
  digitalWrite(leds[2][3], LOW);
  digitalWrite(leds[2][1], LOW);
  digitalWrite(leds[3][5], LOW);
  digitalWrite(leds[3][3], LOW);
  digitalWrite(leds[3][2], LOW);
  digitalWrite(leds[3][1], LOW);

}
