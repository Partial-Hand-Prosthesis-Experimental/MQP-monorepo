#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>
#include <MiniPID.h>
#include <FreeRTOS.h>
#include "BLEProp.h"
class Motor {
    public:
        Motor (int pinA1, int pinA2, volatile int* potReading, volatile int* currentReading);
        Motor (int pinA1, int pinA2, volatile int* potReading, volatile int* currentReading, BLEProp* prop);
        void speed(float_t speed);
        void position(float_t position);
        BLEProp* _prop;
    private:
        volatile int* _potReading;
        int _pinA1;
        int _pinA2;
        MiniPID _pid = MiniPID(0.0, 0.0, 0.0);
};
#endif