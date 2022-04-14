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
#include <SimpleKalmanFilter.h>
#include <Arduino_KNN.h>
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
#define TIMER0_INTERVAL_US 50

void IRAM_ATTR TimerHandler0();
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t *timer = NULL;

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

// Headers
int doBrian(bool debug_prints);
void calib_switch();
float adc2v(int adc_val);

// Brian globals
// TODO Fix all of these. All are from arduino nano
// Pins
const int buttonPin = 2;
const int buttonPin2 = 3;
const int calib_button_pin = 4; // D4
// const int potPin = 1; we both used pot pin
int LED_pin = 2; // D2

const int Hall_1_Pin = 0;
const int Hall_2_Pin = 0;
const int Hall_3_Pin = A3;
const int Hall_4_Pin = A4;
const int Hall_5_Pin = A5;
const int Hall_6_Pin = A6;
const int Hall_7_Pin = A7;

// States
int buttonState = 0;
int buttonState2 = 0;
int calib_button_push = 0;
enum State
{
  standby,
  calibration,
  motoring
};
State s = standby;

// Global Consts
const int sensor_num = 6;
const int interval_duration = 2000; // ms
const int intervals = 12;
const int samples_per_interval = 125;
const int time_per_sample = interval_duration / samples_per_interval;
const int calib_duration = interval_duration * intervals;
const int total_samples = intervals * samples_per_interval;

// GLobal Vars
int calib_i = 0;

// Timers
// TODO Refactor these with drews timer
unsigned long start_time;
unsigned long last_time;
unsigned long current_time;
unsigned long elapsed_time;

// Kalman Filter
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);
// KNN Clasifier
KNNClassifier myKNN(6);//same as sensor_num

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

  // Brian Setup
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(calib_button_pin, INPUT);
  pinMode(LED_pin, OUTPUT);

  // Kalman Filter Setup

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
    // Serial.println(time);
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
    delay(500); // give the bluetooth stack the chance to get things ready
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

  doBrian(true);
}

// Currently Runs at 20khz, see #define TIMER0_INTERVAL_US        50
void IRAM_ATTR TimerHandler0()
{
  portENTER_CRITICAL_ISR(&timerMux);
  potReading = analogRead(potPin);
  currentReading = analogRead(currentPin);
  portEXIT_CRITICAL_ISR(&timerMux);
}

// Brian helper
float adc2v(int adc_val)
{
  float source_voltage = 4.3;
  int resolution = 1023;
  float voltage = (adc_val * source_voltage) / (resolution);
  return voltage;
}

void calib_switch()
{
  Serial.println("switching to calibration");
  s = calibration;
  start_time = millis();
  last_time = millis();
  calib_i = 0;

  digitalWrite(LED_pin, HIGH);
  delay(interval_duration * .05);
  digitalWrite(LED_pin, LOW);
  delay(interval_duration * .45);

  digitalWrite(LED_pin, HIGH);
  delay(interval_duration * .05);
  digitalWrite(LED_pin, LOW);
  delay(interval_duration * .45);

  digitalWrite(LED_pin, HIGH);
  delay(interval_duration * .05);
  digitalWrite(LED_pin, LOW);
  delay(interval_duration * .45);
}

int doBrian(bool debug_prints)
{
  static unsigned long lastTime = micros();
  if (micros() - lastTime > 2000000)
  {
    Serial.print("Brian Interval: ");
    Serial.println(micros() - lastTime);
    lastTime = micros();
  }

  int summeded_data[samples_per_interval][6];
  switch (s)
  {
  case standby:
    //      Serial.println("standby");
    calib_button_push = digitalRead(calib_button_pin);
    if (calib_button_push)
    {
      if(debug_prints){Serial.println("PUSHED");}
      calib_switch();
    }
    break;

  case calibration:
    // calibrate, storing sum in averaged_data
    if (elapsed_time < calib_duration)
    {
      calib_i++;
      // delay sampling to achive desired rate
      while (millis() < (elapsed_time + time_per_sample))
      {
        Serial.print("delayed");
        delay(1);
      }
      int hall_1 = analogRead(Hall_1_Pin);
      int hall_2 = analogRead(Hall_2_Pin);
      int hall_3 = analogRead(Hall_3_Pin);
      int hall_4 = analogRead(Hall_4_Pin);
      int hall_5 = analogRead(Hall_5_Pin);
      int hall_6 = analogRead(Hall_6_Pin);

      current_time = millis();
      elapsed_time = current_time - start_time;

      bool opening = (int)floor((int)elapsed_time / (int)interval_duration) % 2 == 0;
      int pos = 0;

      // sum readings with respoect to position
      if (opening)
      {
        digitalWrite(LED_pin, HIGH);

        pos = calib_i % samples_per_interval;
        summeded_data[pos][0] += hall_1;
        summeded_data[pos][1] += hall_2;
        summeded_data[pos][2] += hall_3;
        summeded_data[pos][3] += hall_4;
        summeded_data[pos][4] += hall_5;
        summeded_data[pos][5] += hall_6;
      }
      else
      { // count position backwards if closing
        digitalWrite(LED_pin, LOW);

        summeded_data[samples_per_interval - calib_i % samples_per_interval][0] += hall_1;
        summeded_data[samples_per_interval - calib_i % samples_per_interval][1] += hall_2;
        summeded_data[samples_per_interval - calib_i % samples_per_interval][2] += hall_3;
        summeded_data[samples_per_interval - calib_i % samples_per_interval][3] += hall_4;
        summeded_data[samples_per_interval - calib_i % samples_per_interval][4] += hall_5;
        summeded_data[samples_per_interval - calib_i % samples_per_interval][5] += hall_6;
      }
      if (debug_prints) // Print current reading
      {
        Serial.print(elapsed_time);
        Serial.print(", ");
        Serial.print(hall_1);
        Serial.print(", ");
        Serial.print(hall_2);
        Serial.print(", ");
        Serial.print(hall_3);
        Serial.print(", ");
        Serial.print(hall_4);
        Serial.print(", ");
        Serial.print(hall_5);
        Serial.print(", ");
        Serial.print(hall_6);
        Serial.print(", ");
        Serial.println((int)floor((int)elapsed_time / (int)interval_duration));
      }

      if ((int)floor((int)elapsed_time / (int)interval_duration) % 2 == 0)
      {
        digitalWrite(LED_pin, HIGH);
      }
      else
      {
        digitalWrite(LED_pin, LOW);
      }
    }
    else
    {
      for (int b = 0; b < samples_per_interval; b++)
      {
        float averaged_data[samples_per_interval][sensor_num];

        averaged_data[b][0] = summeded_data[b][0] / intervals;
        averaged_data[b][1] = summeded_data[b][1] / intervals;
        averaged_data[b][2] = summeded_data[b][2] / intervals;
        averaged_data[b][3] = summeded_data[b][3] / intervals;
        averaged_data[b][4] = summeded_data[b][4] / intervals;
        averaged_data[b][5] = summeded_data[b][5] / intervals;
        if (debug_prints)
        {
          Serial.print("Average entry at position ");
          Serial.print(b);
          Serial.print("/");
          Serial.print(samples_per_interval);
          for (int a = 0; a < 7; a++)
          {
            Serial.print(averaged_data[b][a]);
          }
          Serial.println("");
        }
        myKNN.addExample(averaged_data[b], b);
      }
      Serial.println("Done calibrating.");
      elapsed_time = 0;
      s = standby; // TODO make motoring
      digitalWrite(LED_pin, LOW);
      break;
    case motoring:
      Serial.println("Motoring.");

      int hall_1 = analogRead(Hall_1_Pin);
      int hall_2 = analogRead(Hall_2_Pin);
      int hall_3 = analogRead(Hall_3_Pin);
      int hall_4 = analogRead(Hall_4_Pin);
      int hall_5 = analogRead(Hall_5_Pin);
      int hall_6 = analogRead(Hall_6_Pin);
      
      //K Nearest Neighbor
      int k = 5;
      float input[] = {adc2v(hall_1), adc2v(hall_2), adc2v(hall_3), adc2v(hall_4), adc2v(hall_5), adc2v(hall_6)};
      int position = myKNN.classify(input, k); // classify input with K=3
      float confidence = myKNN.confidence();
      if (debug_prints)
      {
        Serial.print("position = ");
        Serial.println(position);

        Serial.print("confidence     = ");
        Serial.println(confidence);
      }
      // TODO use confidence to set noise (q)
      float estimated_pos = pressureKalmanFilter.updateEstimate(position);
      return estimated_pos;
    }


    calib_button_push = digitalRead(calib_button_pin);
    if (calib_button_push)
    {
      calib_switch();
    }
    break;
  }
}