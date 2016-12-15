#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#ifdef _WIN32
#include <Windows.h>
#define COM_PORT 4
#else
#include <unistd.h>
#define COM_PORT 24
#endif

#define BD_RATE 9600

#include "rs232.h"

using namespace std;

//serial portu initilize eder
bool init_serial();
/**
 * serial porttan buffer okur.
 * @param buf: karakter bufferının adresi
 * @param size: okunmak istenen boyut
 */
bool readBuf(char **buf, int size);
/**
 * 
 * @param buf karakter bufferı
 * @param size gönderilmek istenen verinin boyutu
 */
bool sendBuf(char *buf, int size);

/**
 * @param pid pid katsayılarını içeren array
 * indexler:
 * 0 -> Px
 * 1 -> Ix
 * 2 -> Dx
 * 3 -> Py
 * 4 -> Iy
 * 5 -> Dy
 */
bool sendPID(float[] pid);

int main(){
	
	

	
	
	
	
	return 0;
}

bool init_serial(){
	
	char mode[]={'8','N','1',0};

	if(RS232_OpenComport(COM_PORT, BD_RATE, mode))
		
		return false;
	
	return true;	
}

bool readBuf(char **buf, int size){
	
	
	/*	TODO gelen veri yoksa false döndür çık
	 * 
	 */
	int n=0;
	
	while(n != size)
		
		n += RS232_PollComport(cport_nr, (*buf)+n, 1);
	
	return true;
}

bool sendBuf(char *buf, int size){
	
	RS232_SendBuf(COM_PORT, buf, size);
	RS232_flushTX(COM_PORT);
	
	return true; //TODO
}

bool sendPID(float[] pid){
	
	return sendBuf((char*)pid, sizeof(float)*6);	
}
