#include <ESP32Servo.h>
#include <MiniPID.h>
class Motor {
    public:
        Motor (int pin1, int pin2);
        void speed(float_t speed);
    private:
        Servo servoFW;
        Servo servoBW;
        int potReading;
};