#ifndef RC_h
#define RC_h

#include "Arduino.h"

class RC
{
	public:
		//Constructor
		RC(byte pin1 = 0, float scaleFactor1 = 1, int pin2 = 0, float scaleFactor2 = 1, int pin3 = 0, float scaleFactor3 = 1);

                //Getters
		float scaledCommand1(), scaledCommand2(), scaledCommand3();
};

#endif
