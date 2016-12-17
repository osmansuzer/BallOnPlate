#include <stdint.h>

float Kp, Ki, Kd, Kp1, Ki1, Kd1;

void setup(){

  Serial.begin(9600);

}

void loop(){
  setDesiredPosition();
}
char buf[30];

void setDesiredPosition(){

  if(Serial.available() == false)
    return ;
     
  Serial.println("foo");

  int n=0, incoming_size=0;
  
  while(n!=1)

    n+=Serial.readBytes(buf, 1);

  Serial.println(*buf);
  
  switch(*buf){

    case '0':
      incoming_size = 9;
      break;
    case '1':
      incoming_size = 25;
      break;
  }
  
  while(n!=incoming_size)

    n+=Serial.readBytes(buf+n, 1);

  Serial.println("foo2");

  switch(*buf){

    case '0':
     
      break;
    case '1':
      memcpy(&Kp, buf+1, 4);
      memcpy(&Ki, buf+5, 4);
      memcpy(&Kd, buf+9, 4);
      memcpy(&Kp1, buf+13, 4);
      memcpy(&Ki1, buf+17, 4);
      memcpy(&Kd1, buf+21, 4);

      Serial.println(Kp);
      Serial.println(Ki);
      Serial.println(Kd);
      Serial.println(Kp1);
      Serial.println(Kd1);
      Serial.println(Ki1);
      
      
      
      break;
  }

}

