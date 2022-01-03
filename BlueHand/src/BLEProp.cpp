#include <BLEProp.h>

BLEProp::BLEProp(const char *uuidS, const char *uuidC, const float min, const float max)
{
    uuidService = uuidS;
    uuidCharacteristic = uuidC;
}

void BLEProp::setCallbacks(BLECharacteristicCallbacks * callbacks) {
    pCharacteristic->setCallbacks(callbacks);
}

void BLEProp::start(){
    pService->start();
}

void BLEProp::setValue(float value) {
    pCharacteristic->setValue(value);
}

void BLEProp::notify() {
    pCharacteristic->notify();
}

void BLEProp::attach(BLEServer * pServer) {
    pService = pServer->createService(uuidService);

    // Create a BLE Characteristic
    pCharacteristic = pService->createCharacteristic(
        uuidCharacteristic,
        BLECharacteristic::PROPERTY_NOTIFY);
          

    pCharacteristic->addDescriptor(new BLE2902());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->addServiceUUID(uuidService);
}