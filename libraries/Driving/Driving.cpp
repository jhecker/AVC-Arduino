#include <Driving.h>

//Constructor args are pins for left and right motor speed and direction
Driving::Driving(byte rightSpeedPin, byte rightDirectionA, byte rightDirectionB, byte leftSpeedPin, byte leftDirectionA, byte leftDirectionB) {
    pinMode(rightSpeedPin, OUTPUT);
    pinMode(rightDirectionA, OUTPUT);
    pinMode(rightDirectionB, OUTPUT);
    pinMode(leftSpeedPin, OUTPUT);
    pinMode(leftDirectionA, OUTPUT);
    pinMode(leftDirectionB, OUTPUT);
    _rightSpeedPin = rightSpeedPin;
    _rightDirectionA = rightDirectionA;
    _rightDirectionB = rightDirectionB;
    _leftSpeedPin = leftSpeedPin;
    _leftDirectionA = leftDirectionA;
    _leftDirectionB = leftDirectionB;
}

//Sets motion at speed, args are for left and right side speed
void Driving::drive(int leftSpeed, int rightSpeed) {
    if (leftSpeed >= 0) {
    	digitalWrite(_leftDirectionA, HIGH);
    	digitalWrite(_leftDirectionB, LOW);
    }
    else {
    	digitalWrite(_leftDirectionA, LOW);
    	digitalWrite(_leftDirectionB, HIGH);
    }

    if (rightSpeed >= 0) {
    	digitalWrite(_rightDirectionA, LOW);
    	digitalWrite(_rightDirectionB, HIGH);
    }
    else {
    	digitalWrite(_rightDirectionA, HIGH);
    	digitalWrite(_rightDirectionB, LOW);
    }

    //Set PWM values using speed magnitudes
    analogWrite(_leftSpeedPin, abs(leftSpeed));
    analogWrite(_rightSpeedPin, abs(rightSpeed));
}

//Stops all motor motion
void Driving::stop() {
    digitalWrite(_leftDirectionA, LOW);
    digitalWrite(_leftDirectionB, LOW);
    digitalWrite(_rightDirectionA, LOW);
    digitalWrite(_rightDirectionB, LOW);
}
