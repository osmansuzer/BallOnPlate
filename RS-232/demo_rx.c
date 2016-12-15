
/**************************************************

file: demo_rx.c
purpose: simple demo that receives characters from
the serial port and print them on the screen,
exit the program by pressing Ctrl-C

compile with the command: gcc demo_rx.c rs232.c -Wall -Wextra -o2 -o test_rx

**************************************************/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#ifdef _WIN32
#include <Windows.h>
#define COM_PORT 0
#else
#include <unistd.h>
#define COM_PORT 24
#endif

#include "rs232.h"



int main()
{
  int i, n,
      cport_nr=COM_PORT,        /* /dev/ttyS0 (COM1 on windows) */ //TODO define the port as macro
      bdrate=9600;       /* 9600 baud */

  unsigned char buf[4096];

  char mode[]={'8','N','1',0};

  
  if(RS232_OpenComport(cport_nr, bdrate, mode))
  {
    printf("Can not open comport\n");

    return(0);
  }

  sleep(5);
 
  
	RS232_flushRX(cport_nr);
  
  while(1){
  
	memset(buf, 0, 4096);
	  
	RS232_SendBuf(cport_nr, "0123456789", 10);
	RS232_flushTX(cport_nr);
	
	int n=0;
	
	while(n!=10)
		
		n += RS232_PollComport(cport_nr, buf+n, 1);
	
	puts(buf);
	
	usleep(50000);
		
  }
  
#ifdef _WIN32
    Sleep(100);
#else
    usleep(10000);  /* sleep for 100 milliSeconds */
#endif
  
	
  return(0);
}

