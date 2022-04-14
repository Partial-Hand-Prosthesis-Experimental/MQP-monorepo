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
#include "Motor.h"
#include "Haptics.h"
#include "Preferences.h"

BLEServer *pServer = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

int velo0pin = 15;
int velo1pin = 4;

int potPin = 26;
int currentPin = 27;
volatile int potReading = 0;
volatile int currentReading = 0;

// Timer setup
#define TIMER0_INTERVAL_US        50

void IRAM_ATTR TimerHandler0();
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t * timer = NULL;

Motor motor(25, 33, &potReading, &currentReading);
Haptics haptics(2);

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID_TEST1 "1a6a0dc9-7db0-4e5f-8b48-5122af7d0b73"
#define CHARACTERISTIC_UUID_TEST1 "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define SERVICE_UUID_TEST3 "6e400005-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TEST3 "6e400006-b5a3-f393-e0a9-e50e24dcca9e"
#define SERVICE_UUID_VIBCONF "6e400007-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_VIBCONF "6e400008-b5a3-f393-e0a9-e50e24dcca9e"
#define SERVICE_UUID_VELO1 "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_VELO1 "6e400004-b5a3-f393-e0a9-e50e24dcca9e"
#define SERVICE_UUID_VELO2 "6e400009-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_VELO2 "6e40000a-b5a3-f393-e0a9-e50e24dcca9e"

#define SERVICE_UUID_VELO2 "6e40000b-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_VELO2 "6e40000d-b5a3-f393-e0a9-e50e24dcca9e"


BLEProp test1(SERVICE_UUID_TEST1, CHARACTERISTIC_UUID_TEST1, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, 4);
BLEProp test3(SERVICE_UUID_TEST3, CHARACTERISTIC_UUID_TEST3, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, 4);
BLEProp velo1(SERVICE_UUID_VELO1, CHARACTERISTIC_UUID_VELO1, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, 4);
BLEProp velo2(SERVICE_UUID_VELO2, CHARACTERISTIC_UUID_VELO2, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, 4);
BLEProp vibConf(SERVICE_UUID_VIBCONF, CHARACTERISTIC_UUID_VIBCONF, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, 20);

Preferences preferences;

void doBrian();

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
  // BLEDevice::setMTU(64); // Wish this worked but I don't think it does on windows
  pServer = BLEDevice::createServer();

  test1.attach(pServer);
  test3.attach(pServer);
  vibConf.attach(pServer);
  velo1.attach(pServer);
  velo2.attach(pServer);

  Serial.println("Server initialized");

  // Start advertising
  pServer->getAdvertising()->setScanResponse(true);
  pServer->getAdvertising()->start();

  vibConf.setBytes((uint8_t *)&vibConfStart, 8);

  test1.setValue(100.0);
  test3.setValue(0.0);
  velo1.setValue(0.0);
  velo2.setValue(0.0);

  vibConf.notify();
  velo1.notify();
  velo2.notify();
  test1.notify();
  test3.notify();

  Serial.println("Waiting a client connection to notify...");


  // Setup timer
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &TimerHandler0, true);
  timerAlarmWrite(timer, 500, true);
  timerAlarmEnable(timer);

}

void loop()
{
  if (deviceConnected)
  {
    float time = (float)millis();
    //Serial.println(time);
    motor.position(test3.getFloat());
    test1.setValue(potReading);
    test1.notify();
    velo1.setValue((float)65.7 * powf(analogReadMilliVolts(velo0pin) / 1000.0, -1.35));
    velo1.notify();
    velo2.setValue((float)65.7 * powf(analogReadMilliVolts(velo1pin) / 1000.0, -1.35));
    velo2.notify();
    test3.notify();
    vibConf.notify();
    delay(10); // bluetooth stack will go into congestion if too many packets are sent
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->disconnect(0);
    pServer->getAdvertising()->setScanResponse(true);
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
      haptics.vibeselect(i);
      Serial.print("go on haptic #");
      Serial.println(i);
      haptics.drv->go();
    }

    if ((float)65.7 * powf(analogReadMilliVolts(velo0pin) / 1000.0, -1.35) > 300)
    {
      haptics.vibeselect(0);
      haptics.drv->go();
    }
    if ((float)65.7 * powf(analogReadMilliVolts(velo1pin) / 1000.0, -1.35) > 300)
    {
      haptics.vibeselect(1);
      haptics.drv->go();
    }
  }

  doBrian();
}

void doBrian() {
    static unsigned long lastTime = micros();
    if(micros() - lastTime> 2000000) {
      Serial.print("Brian Interval: ");
      Serial.println(micros()-lastTime);
      lastTime = micros();
    }
}

// Currently Runs at 20khz, see #define TIMER0_INTERVAL_US        50
void IRAM_ATTR TimerHandler0()
{
    portENTER_CRITICAL_ISR(&timerMux);
    potReading = analogRead(potPin);
    currentReading = analogRead(currentPin);
    portEXIT_CRITICAL_ISR(&timerMux);
}