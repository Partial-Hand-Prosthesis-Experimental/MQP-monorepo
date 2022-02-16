#include <BLEProp.h>

BLEProp::BLEProp(const char *uuidS, const char *uuidC, uint32_t properties, int8_t _byteCount)
{
    uuidService = uuidS;
    uuidCharacteristic = uuidC;
    characteristicProperties = properties;
    byteCount = _byteCount;
}

void BLEProp::setCallbacks(BLECharacteristicCallbacks *callbacks)
{
    pCharacteristic->setCallbacks(callbacks);
}

void BLEProp::start()
{
    pService->start();
}

void BLEProp::setValue(float value)
{
    pCharacteristic->setValue(value);
}

void BLEProp::setBytes(uint8_t *bytes, size_t len)
{
    pCharacteristic->setValue(bytes, len);
}

const char *BLEProp::getStr()
{
    return pCharacteristic->getValue().c_str();
}

float BLEProp::getFloat()
{
    int byteIdx = 0;
    FloatConversion conversion;
    for (byteIdx; byteIdx < 4; byteIdx++)
    {
        conversion.byte[byteIdx] = pCharacteristic->getValue().c_str()[byteIdx];
    }

    return conversion.f;
}

uint8_t *BLEProp::getData()
{
    return pCharacteristic->getData();
}

void BLEProp::notify()
{
    pCharacteristic->notify();
}

void BLEProp::attach(BLEServer *pServer)
{
    pService = pServer->createService(uuidService);

    // Create a BLE Characteristic
    pCharacteristic = pService->createCharacteristic(
        uuidCharacteristic,
        characteristicProperties);

    Serial.println(pCharacteristic->PROPERTY_NOTIFY | pCharacteristic->PROPERTY_WRITE | pCharacteristic->PROPERTY_READ);
    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->addDescriptor(new BLE2904());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->addServiceUUID(uuidService);
}