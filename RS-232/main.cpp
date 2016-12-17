#include "serial.h"
#include <iostream>
#include <unistd.h>
#include <signal.h>

using namespace std;
/*
int main(){
	
	char mode[]={'8','N','1',0};

	if(RS232_OpenComport(COM_PORT, BD_RATE, mode) == 1){
		
		fprintf(stderr, "init error\n");
		exit(1);
	}
	
	sleep(4);
	
	int n = 0;
	unsigned char buf[6]="hello";

	int sent = RS232_SendBuf(COM_PORT, buf, 5);
	
	fprintf(stderr, "sent: %d\n", sent);
	
	n=0;
	while(n<6)
		
		n += RS232_PollComport(COM_PORT, buf+n, 1);
	
	puts((const char*)buf);	
	
	RS232_CloseComport(COM_PORT);
}*/




void foo(int sig){
	
	printf("signal\n");
 	RS232_CloseComport(COM_PORT);
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
	sleep(4);
	fprintf(stderr, "entered loop\n");

 	float arr[]={1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
 	
  	unsigned char buf[1];
  int a = 0;


	a = RS232_SendBuf(COM_PORT, (unsigned char*)"1", 1);
	a += RS232_SendBuf(COM_PORT, (unsigned char*)arr, sizeof(float) *6);

	fprintf(stderr,"%d\n",a);
	fprintf(stderr, "entered loop2\n");
	
	while(1){
		
		
		if(RS232_PollComport(COM_PORT, buf, 1) > 0)
			printf("%c", *buf);
		
	}
	
	return 0;
}
