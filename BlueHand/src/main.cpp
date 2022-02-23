/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini
   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"
   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.
   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second. 
*/
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEProp.h>
#include <Wire.h>
#include "Adafruit_DRV2605.h"
#include "Preferences.h"

BLEServer *pServer = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;
int velo0pin = 15;
int velo1pin = 4;


Adafruit_DRV2605 drv0;
Adafruit_DRV2605 drv1;
Adafruit_DRV2605 drv2;
Adafruit_DRV2605 *drv;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID_TEST1 "1a6a0dc9-7db0-4e5f-8b48-5122af7d0b73" // UART service UUID
#define CHARACTERISTIC_UUID_TEST1 "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define SERVICE_UUID_TEST2 "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TEST2 "6e400004-b5a3-f393-e0a9-e50e24dcca9e"
#define SERVICE_UUID_TEST3 "6e400005-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TEST3 "6e400006-b5a3-f393-e0a9-e50e24dcca9e"
#define SERVICE_UUID_VIBCONF "6e400007-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_VIBCONF "6e400008-b5a3-f393-e0a9-e50e24dcca9e"

#define TCAADDR 0x70

void vibeselect(uint8_t i)
{
  if (i > 7)
    return;

  switch (i)
  {
  case 0:
    drv = &drv0;
    break;
  case 1:
    drv = &drv1;
    break;
  case 2:
    drv = &drv2;
    break;
  }

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

BLEProp test1(SERVICE_UUID_TEST1, CHARACTERISTIC_UUID_TEST1, BLECharacteristic::PROPERTY_NOTIFY, 4);
BLEProp velo1(SERVICE_UUID_TEST2, CHARACTERISTIC_UUID_TEST2, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, 4);
BLEProp test3(SERVICE_UUID_TEST3, CHARACTERISTIC_UUID_TEST3, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, 4);
BLEProp vibConf(SERVICE_UUID_VIBCONF, CHARACTERISTIC_UUID_VIBCONF, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, 20);

Preferences preferences;

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0)
    {
    }
  }
};

void setup()
{
  Serial.begin(115200);
  uint8_t vibConfStart[] = {
      255,
      255,
      0,
      0,
      0,
      127,
      63,
      31};
  preferences.begin("vibro", false);
  // preferences.getBytes("vibConf", vibConfStart, 8U);
  preferences.putBytes("vibConf", vibConfStart, sizeof(vibConfStart));

  // Create the BLE Device
  BLEDevice::init("Prosthesis");
  pServer = BLEDevice::createServer();

  test1.attach(pServer);
  velo1.attach(pServer);
  test3.attach(pServer);
  vibConf.attach(pServer);

  Serial.println("Server initialized");
  // Create the BLE Server
  pServer->setCallbacks(new MyServerCallbacks());

  //test1.setCallbacks(new MyCallbacks());

  // Start advertising
  pServer->getAdvertising()->setScanResponse(true);
  pServer->getAdvertising()->start();

  vibConf.setBytes((uint8_t *)&vibConfStart, 8);

  Wire.begin();
  for (int i = 0; i < 3; i++)
  {
    vibeselect(i);

    //drv setup
    drv->begin();

    // I2C trigger by sending 'go' command
    drv->setMode(DRV2605_MODE_INTTRIG); // default, internal trigger when sending GO command

    drv->selectLibrary(1);
    drv->setWaveform(0, 14); // ramp up medium 1, see datasheet part 11.2
    //drv->setWaveform(1, 47); // strong click 100%, see datasheet part 11.2
    drv->setWaveform(1, 0); // end of waveforms
  }

  test1.setValue(100.0);
  velo1.setValue(2.4);
  test3.setValue(0.01);

  vibConf.notify();
  test1.notify();
  velo1.notify();
  test3.notify();

  Serial.println("Waiting a client connection to notify...");
}

void loop()
{

  if (deviceConnected)
  {
    float time = (float)millis();
    //Serial.println(time);
    test1.setValue((float)sin(time / 1000.0 * test3.getFloat()));
    test1.notify();
    velo1.setValue((float)65.7 * powf(analogReadMilliVolts(velo0pin) / 1000.0, -1.35));
    velo1.notify();
    test3.notify();
    vibConf.notify();
    delay(10); // bluetooth stack will go into congestion if too many packets are sent
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
    Serial.println("Client Disconnected");
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    Serial.println("Client Connected");
  }

  for (int i = 0; i < 3; i++)
  {
    if (vibConf.getData()[2 + i] == 1)
    {
      uint8_t *vibUpdate = vibConf.getData();
      vibUpdate[2 + i] = 0;
      vibConf.setBytes(vibUpdate, 8);
      vibeselect(i);
      Serial.print("go on haptic #");
      Serial.println(i);
      drv->go();
    }

    if ((float)65.7 * powf(analogReadMilliVolts(velo0pin) / 1000.0, -1.35) > 300)
    {
      vibeselect(0);
      drv->go();
    }
    if ((float)65.7 * powf(analogReadMilliVolts(velo1pin) / 1000.0, -1.35) > 300)
    {
      vibeselect(1);
      drv->go();
    }
  }
}
