#ifndef Driving_h
#define Driving_h

#include "Arduino.h"

class Driving
{
	public:
		//Constructors
		Driving(byte rightSpeedPin, byte rightDirectionA, byte rightDirectionB, byte leftSpeedPin, byte leftDirectionA, byte leftDirectionB, float wheelBase);
		
		//Functions
		void commandVelocity(float linearVelocity, float angularVelocity);
		
	private:
		void drive(int leftSpeed, int rightSpeed);
		byte _rightSpeedPin, _rightDirectionA, _rightDirectionB, _leftSpeedPin, _leftDirectionA, _leftDirectionB;
		float _wheelBase;
};

#endif
