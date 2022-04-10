#ifndef HAPTICS_H
#define HAPTICS_H
#include <Arduino.h>
#include "Adafruit_DRV2605.h"
#include <Wire.h>

#define TCAADDR 0x70

class Haptics {
    public:
        Haptics(int count);
        void vibeselect(uint8_t i);
        Adafruit_DRV2605 *drv;
    private:
        Adafruit_DRV2605 drv0;
        Adafruit_DRV2605 drv1;
        Adafruit_DRV2605 drv2;
};

#endif