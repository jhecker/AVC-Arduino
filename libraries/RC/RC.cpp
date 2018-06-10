#define LIBCALL_ENABLEINTERRUPT
#include <RC.h>
#include <EnableInterrupt.h>

#define MEAN_PULSE_LENGTH 1500
#define MAX_PULSE_LENGTH 2000

byte _pin1, _pin2, _pin3;
float _scaleFactor1, _scaleFactor2, _scaleFactor3;
float _scaledCommand1, _scaledCommand2, _scaledCommand3;
void ISR1(), ISR2(), ISR3();

//Constructor args are pins for linear and angular interrupts
RC::RC(byte pin1 = 0, float scaleFactor1 = 1, int pin2 = 0, float scaleFactor2 = 1, int pin3 = 0, float scaleFactor3 = 1) {
    if (pin1 > 0) {
    	enableInterrupt(pin1, ISR1, CHANGE);
    	_pin1 = pin1;
    	_scaleFactor1 = scaleFactor1; 
    }
    if (pin2 > 0) {
    	enableInterrupt(pin2, ISR2, CHANGE);
    	_pin2 = pin2;
    	_scaleFactor2 = scaleFactor2; 
    }
    if (pin3 > 0) {
    	enableInterrupt(pin3, ISR3, CHANGE);
    	_pin3 = pin3;
    	_scaleFactor3 = scaleFactor3; 
    }
}

//Getters
float RC::scaledCommand1() {
    return _scaledCommand1;
}

float RC::scaledCommand2() {
    return _scaledCommand2;
}

float RC::scaledCommand3() {
    return _scaledCommand3;
}

//Interrupt service routines (ISRs) for each pin
void ISR1() {
    static unsigned long lastInterruptTime = 0;
    if (digitalRead(_pin1) == HIGH) {
    	lastInterruptTime = micros();
    }
    else if (lastInterruptTime > 0) {
        float period = micros() - lastInterruptTime;
        //normalize pulse
        _scaledCommand1 = ((period - MEAN_PULSE_LENGTH) / (MAX_PULSE_LENGTH - MEAN_PULSE_LENGTH)) * _scaleFactor1;
    }
}

void ISR2() {
    static unsigned long lastInterruptTime = 0;
    if (digitalRead(_pin2) == HIGH) {
    	lastInterruptTime = micros();
    }
    else if (lastInterruptTime > 0) {
        float period = micros() - lastInterruptTime;
        //normalize pulse
        _scaledCommand2 = ((period - MEAN_PULSE_LENGTH) / (MAX_PULSE_LENGTH - MEAN_PULSE_LENGTH)) * _scaleFactor2;
    }
}

void ISR3() {
    static unsigned long lastInterruptTime = 0;
    if (digitalRead(_pin3) == HIGH) {
    	lastInterruptTime = micros();
    }
    else if (lastInterruptTime > 0) {
        float period = micros() - lastInterruptTime;
        //normalize pulse
        _scaledCommand3 = ((period - MEAN_PULSE_LENGTH) / (MAX_PULSE_LENGTH - MEAN_PULSE_LENGTH)) * _scaleFactor3;
    }
}
