#include "serial.h"
#include <iostream>
#include <unistd.h>
#include <signal.h>

using namespace std;

void foo(int sig){
	
	printf("signal\n");
 //	RS232_CloseComport(COM_PORT);
	exit(0);
	
}

int main(){
/*		
	int16_t x,  y;
	float servo_x, servo_y;*/

	signal(SIGINT, foo);
	
	if(!init_serial()){
 		
 		fprintf(stderr, "init err");
 		exit(1);
 	}
	
	fprintf(stderr, "entered loop\n");

 	float arr[]={1.0, 3.0, 1.0, 1.0, 1.0, 1.0};
 	
  	unsigned char buf[1];
  int a;

	a = RS232_SendBuf(COM_PORT, (unsigned char*)"1", 1);
	RS232_SendBuf(COM_PORT, (unsigned char*)arr, sizeof(float) *6);

	fprintf(stderr,"%d",a);
	fprintf(stderr, "entered loop2\n");
	
	while(1){
		
		
		if(RS232_PollComport(COM_PORT, buf, 1) > 0)
			printf("%c", *buf);
		
	}
	
	
	return 0;
}
