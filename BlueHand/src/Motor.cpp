#include "Motor.h"

Motor::Motor(int pinA1, int pinA2, int pinPot) {
    _pinA1 = pinA1;
    _pinA2 = pinA2;
    _pinPot = pinPot;
    // Setup the pwm channels
    ledcSetup(0, 20000, 16);
    ledcAttachPin(_pinA1, 0);
    ledcSetup(1, 20000, 16);
    ledcAttachPin(_pinA2, 1);

    // Setup minipid
    _pid.reset();
    _pid.setPID(0.0005, 0.0005, 0.00001);
    _pid.setOutputLimits(-0.8, 0.8);
}

Motor::Motor(int pinA1, int pinA2, int pinPot, BLEProp prop) {
    Motor(pinA1, pinA2, pinPot);
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

void Motor::position(float_t position) {
    // sensor, target
    float_t out = (float_t)_pid.getOutput(analogRead(_pinPot), position);
    speed(out);
}