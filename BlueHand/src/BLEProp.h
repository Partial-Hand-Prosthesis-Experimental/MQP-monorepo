#ifndef BLEProp_H
#define BLEProp_H
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLE2904.h>
#include <Arduino.h>

union FloatConversion
{
    float f;
    unsigned char byte[4];
};

class BLEProp
{
public:
    BLEProp(const char *uuidS, const char *uuidC, uint32_t properties, size_t _byteCount);
    void setCallbacks(BLECharacteristicCallbacks *callbacks);
    void start();
    void notify();
    void setValue(float value);
    void setBytes(uint8_t *bytes, size_t len);
    uint8_t *getData();
    const char *getStr();
    float getFloat();
    void attach(BLEServer *pServer);
    size_t byteCount;

private:
    const char *uuidService;
    const char *uuidCharacteristic;
    BLEService *pService;
    BLECharacteristic *pCharacteristic;
    uint32_t characteristicProperties;
};
#endif