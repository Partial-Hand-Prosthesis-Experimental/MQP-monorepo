#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>
#include <MiniPID.h>
#include "BLEProp.h"
class Motor {
    public:
        Motor (int pinA1, int pinA2, int pinPot);
        Motor (int pinA1, int pinA2, int pinPot, BLEProp prop);
        void speed(float_t speed);
        void position(float_t position);
        BLEProp _prop;
    private:
        int potReading;
        int _pinA1;
        int _pinA2;
        int _pinPot;
        MiniPID _pid = MiniPID(0.0, 0.0, 0.0);
};
#endif