

#ifndef _arduino_nano_h
#define _arduino_nano_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif



#define msg_toAPM_size 4
#define msg_fromAPM_size 9


// byte = unsigned char

byte apmRXi = 0;
byte msg_toAPM[msg_toAPM_size] = {0};
byte msg_fromAPM[msg_fromAPM_size] = {0};


#endif

