#ifndef Driving_h
#define Driving_h

#include "Arduino.h"

class Driving
{
	public:
		//Constructors
		Driving(byte rightSpeedPin, byte rightDirectionA, byte rightDirectionB, byte leftSpeedPin, byte leftDirectionA, byte leftDirectionB);
		
		//Functions
		void drive(int leftSpeed, int rightSpeed);
		void stop();
		
	private:
        byte _rightSpeedPin, _rightDirectionA, _rightDirectionB, _leftSpeedPin, _leftDirectionA, _leftDirectionB;
};

#endif
