#include "Motor.h"

Motor::Motor(int pin1, int pin2) {
    servoFW.attach(pin1, 0, 20000);
    servoBW.attach(pin1, 0, 20000);
}

void Motor::speed(float_t speed) {
    if(speed > 0) {
        int ang = (int)(constrain(speed*180, 0, 180));
        servoBW.writeMicroseconds(0);
        servoFW.write(ang);
    }
    else {
        int ang = (int)(constrain(speed*180, 0, 180));
        servoFW.writeMicroseconds(0);
        servoBW.write(ang);
    }
}

