#include <stdlib.h>
#include <stdio.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "rs232.h"

#define COM_PORT 24

int main(){
	
	char mode[]={'8','N','1',0};

	RS232_OpenComport(COM_PORT, 9600, mode);
	
	float arr[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
	
	RS232_SendByte(COM_PORT, '1');
	
	RS232_SendBuf(COM_PORT, (char*)arr, 24);
	
	RS232_SendByte(COM_PORT, '1');
	
	RS232_SendBuf(COM_PORT, (char*)arr, 24);
	
	
	char buf[1];
	
	fprintf(stderr, "loop\n");
	while(1){
		
		if(RS232_PollComport(COM_PORT, buf, 1) == 1)
			
			putchar(*buf);
		
		
	}
	
	
	
	
	
	
	
	
	return 0;
}
