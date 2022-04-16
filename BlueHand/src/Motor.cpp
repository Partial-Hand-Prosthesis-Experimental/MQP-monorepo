#include "Motor.h"

Motor::Motor(int pinA1, int pinA2, volatile int* potReading, volatile int* currentReading) {
    _pinA1 = pinA1;
    _pinA2 = pinA2;
    _potReading = potReading;
    _currentReading = currentReading;
    // Setup the pwm channels
    ledcSetup(0, 40000, 16);
    ledcAttachPin(_pinA1, 0);
    ledcSetup(1, 40000, 16);
    ledcAttachPin(_pinA2, 1);

    // Setup minipid
    _pid.reset();
    _pid.setPID(0.005, 0.000005, 0.0001);
    _pid.setOutputLimits(-1.0, 1.0);
    _currentpid.reset();
    _currentpid.setPID(5, 0.0001, 0.0);
    _currentpid.setOutputLimits(-65535, 65535.0);
}

Motor::Motor(int pinA1, int pinA2, volatile int* potReading, volatile int* currentReading, BLEProp* prop) {
    Motor(pinA1, pinA2, potReading, currentReading);
    _prop = prop;
}

void Motor::speed(float_t speed) {
    int duty = (int)(constrain(abs(speed)*65536, 0, 65536));
    if(speed > 0) {
        ledcWrite(0, duty);
        ledcWrite(1, 0);
    }
    else {
        ledcWrite(0, 0);
        ledcWrite(1, duty);
    }
}

void Motor::current(float_t current, float_t maxCurrent) {
    int maxduty = (int)(constrain(abs(maxCurrent)*65536, 0, 65536));
    _currentpid.setOutputLimits(-maxduty, maxduty);
    int duty = (int)(constrain(abs(current)*65535, 0, 65535));
    int out = (int)_currentpid.getOutput(*_currentReading, duty);
    if(current > 0) {
        ledcWrite(0, out);
        ledcWrite(1, 0);
    }
    else {
        ledcWrite(0, 0);
        ledcWrite(1, out);
    }
}

void Motor::position(float_t position) {
    // sensor, target
    float_t out = (float_t)_pid.getOutput(*_potReading, position);
    current(out, 0.1); // TODO: make max current 0.9*the max voltage dropped by the sense resistor
    // Sense resistors voltage is a function of the battery voltage and the motor resistance + sense resistance
    //speed(out);
}