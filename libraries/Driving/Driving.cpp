#include <Driving.h>

//Constructor args are pins for left and right motor speed and direction
Driving::Driving(byte rightSpeedPin, byte rightDirectionA, byte rightDirectionB, byte leftSpeedPin, byte leftDirectionA, byte leftDirectionB, float wheelBase) {
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
    _wheelBase = wheelBase;
}

//Estimates PWM values for left and right motors given commanded linear and angular velocities
void Driving::commandVelocity(float linearVelocity, float angularVelocity) {
    float leftSpeed = (linearVelocity + (angularVelocity * (_wheelBase / 2.0)));
    float rightSpeed = (linearVelocity - (angularVelocity * (_wheelBase / 2.0)));

    if (leftSpeed > 0) {
      leftSpeed = min(pow(2.0, 2.65 * leftSpeed + 3.6),255);
    }
    else if (leftSpeed < 0) {
      leftSpeed = max(-pow(2.0, 2.65 * abs(leftSpeed) + 3.6),-255);
    }

    if (rightSpeed > 0) {
      rightSpeed = min(pow(2.0, 2.65 * rightSpeed + 3.6),255);
    }
    else if (rightSpeed < 0) {
      rightSpeed = max(-pow(2.0, 2.65 * abs(rightSpeed) + 3.6),-255);
    }

    drive((int)leftSpeed, (int)rightSpeed);
}

//Sets direction and PWM values for left-side and right-side motors
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
