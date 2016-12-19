#include <iostream>
#include <cstdlib>
#include "serial.h"
#include <signal.h>

using namespace std;

void foo(int sig){
	
		RS232_CloseComport(COM_PORT);
		exit(0);
}

int main(){
	
	signal(SIGINT, foo);
	
	if(!init_serial()){
		
		cerr << "init err" << endl;
		exit(1);
	}	
	
	char buf[1];
	
	while(!readBuf(buf, 1))
		;
	
	char *str = "fK";
	
	sendBuf(str, 2);
	
		
	while(!readBuf(buf, 1))
		;
	
	cout << *buf << endl;
	
	
	RS232_CloseComport(COM_PORT);
	
	
	return 0;
}
