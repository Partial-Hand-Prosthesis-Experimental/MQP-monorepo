#ifndef BLEProp_H
#define BLEProp_H
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Arduino.h>


class BLEProp {
    public:
        BLEProp(const char * uuidS, const char * uuidC, const float min, const float max);
        void setCallbacks(BLECharacteristicCallbacks * callbacks);
        void start();
        void notify();
        void setValue(float value);
        void attach(BLEServer * pServer);
    private:
        const char * uuidService;
        const char * uuidCharacteristic;
        BLEService * pService;
        BLECharacteristic * pCharacteristic;
};
#endif