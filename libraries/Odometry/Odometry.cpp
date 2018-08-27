#define LIBCALL_ENABLEINTERRUPT
#include <Odometry.h>
#include <EnableInterrupt.h>

//Global Functions
void rightEncoderAChange();
void rightEncoderBChange();
void leftEncoderAChange();
void leftEncoderBChange();

//Global Variables
long rightEncoderCounter;
long leftEncoderCounter;
long prevRightEncoderCounter;
long prevLeftEncoderCounter;
byte _rightEncoderAPin;
byte _rightEncoderBPin;
byte _leftEncoderAPin;
byte _leftEncoderBPin;

/**
 *	Constructor arg are pins for channels A and B on right and left encoders
 **/
Odometry::Odometry(byte rightEncoderAPin, byte rightEncoderBPin, byte leftEncoderAPin, byte leftEncoderBPin, float wheelBase, float wheelDiameter, int cpr) {
    pinMode(rightEncoderAPin, INPUT);
    pinMode(rightEncoderBPin, INPUT);
    pinMode(leftEncoderAPin, INPUT);
    pinMode(leftEncoderBPin, INPUT);
    digitalWrite(leftEncoderBPin, HIGH);
    rightEncoderCounter = 0.;
    leftEncoderCounter = 0.;
    enableInterrupt(rightEncoderAPin, rightEncoderAChange, CHANGE);
    enableInterrupt(rightEncoderBPin, rightEncoderBChange, CHANGE);
    enableInterrupt(leftEncoderAPin, leftEncoderAChange, CHANGE);
    enableInterrupt(leftEncoderBPin, leftEncoderBChange, CHANGE);
    _rightEncoderAPin = rightEncoderAPin;
    _rightEncoderBPin = rightEncoderBPin;
    _leftEncoderAPin = leftEncoderAPin;
    _leftEncoderBPin = leftEncoderBPin;
    _wheelBase = wheelBase;
    _wheelDiameter = wheelDiameter;
    _cpr = cpr;
    
    theta = 0;
    clock = millis();
}

void Odometry::update() {
    //Update linear distance that each wheel has traveled
    float rightWheelDistance = ((float)rightEncoderCounter / _cpr) * _wheelDiameter * PI;
    float leftWheelDistance = ((float)leftEncoderCounter / _cpr) * _wheelDiameter * PI;

    //Calculate absolute heading
    theta = (rightWheelDistance - leftWheelDistance) / _wheelBase;

    //Decompose linear distance into its component values
    float meanWheelDistance = (rightWheelDistance + leftWheelDistance) / 2;
    x = meanWheelDistance * cos(theta);
    y = meanWheelDistance * sin(theta);

    //Calculate speed
    vr = (rightEncoderCounter - prevRightEncoderCounter) / (millis() - clock) * 1000;
    vl = (leftEncoderCounter - prevLeftEncoderCounter) / (millis() - clock) * 1000;
    
    //Calculate relative angle that robot has turned since last update
    float dtheta = (float)((rightEncoderCounter - prevRightEncoderCounter) - (leftEncoderCounter - prevLeftEncoderCounter)) / _wheelBase;

    //Calculate linear velocity
    vx = vr * cos(dtheta);
    vy = vl * sin(dtheta);

    //Calculate angular velocity
    vtheta = dtheta / (millis() - clock) * 1000;
    
    //Store counters
    prevRightEncoderCounter = rightEncoderCounter;
    prevLeftEncoderCounter = leftEncoderCounter;
    
    //Reset clock
    clock = millis();
}

void rightEncoderAChange() {
    bool rightEncoderAStatus = digitalRead(_rightEncoderAPin);
    bool rightEncoderBStatus = digitalRead(_rightEncoderBPin);
    if (((rightEncoderAStatus == HIGH) && (rightEncoderBStatus == HIGH)) || ((rightEncoderAStatus == LOW) && (rightEncoderBStatus == LOW))) {
        rightEncoderCounter++;
    }
    else {
        rightEncoderCounter--;
    }
}

void rightEncoderBChange() {
    bool rightEncoderAStatus = digitalRead(_rightEncoderAPin);
    bool rightEncoderBStatus = digitalRead(_rightEncoderBPin);
    if (((rightEncoderAStatus == HIGH) && (rightEncoderBStatus == LOW)) || ((rightEncoderAStatus == LOW) && (rightEncoderBStatus == HIGH))) {
        rightEncoderCounter++;
    }
    else {
        rightEncoderCounter--;
    }
}

void leftEncoderAChange() {
    bool leftEncoderAStatus = digitalRead(_leftEncoderAPin);
    bool leftEncoderBStatus = digitalRead(_leftEncoderBPin);
    if (((leftEncoderAStatus == HIGH) && (leftEncoderBStatus == LOW)) || ((leftEncoderAStatus == LOW) && (leftEncoderBStatus == HIGH))) {
        leftEncoderCounter++;
    }
    else {
        leftEncoderCounter--;
    }
}

void leftEncoderBChange() {
    bool leftEncoderAStatus = digitalRead(_leftEncoderAPin);
    bool leftEncoderBStatus = digitalRead(_leftEncoderBPin);
    if (((leftEncoderAStatus == HIGH) && (leftEncoderBStatus == HIGH)) || ((leftEncoderAStatus == LOW) && (leftEncoderBStatus == LOW))) {
        leftEncoderCounter++;
    }
    else {
        leftEncoderCounter--;
    }
}