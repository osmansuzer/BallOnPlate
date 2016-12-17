#include "serial.h"
#include <iostream>
#include <signal.h>

using namespace std;

void foo(int sig){
	
 RS232_CloseComport(COM_PORT)	;
 exit(0);
}

int main(){
		
	int16_t x,  y;
	float servo_x, servo_y;
	
	signal(SIGINT, foo);

	if(!init_serial()){
		
		fprintf(stderr, "init err");
		exit(1);
	}
	
	fprintf(stderr, "entered loop\n");
	
	while(1){
	
		if(getCoordinates(&x, &y, &servo_x, &servo_y))
			
			cout << x << " " << y << " " << servo_x << " " << servo_y << endl;
	}
	return 0;
}
