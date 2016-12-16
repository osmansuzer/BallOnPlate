#include "serial.h"
#include <iostream>

using namespace std;

int main(){
		
	int16_t x,  y;
	float servo_x, servo_y;

	if(!init_serial()){
		
		fprintf(stderr, "init err");
		exit(1);
	}
	
	RS232_flushRX(COM_PORT);
	
	fprintf(stderr, "entered loop\n");
	
	while(1){
	
		if(getCoordinates(&x, &y, &servo_x, &servo_y))
			
			cout << x << " " << y << " " << servo_x << " " << servo_y << endl;
	}
	return 0;
}
