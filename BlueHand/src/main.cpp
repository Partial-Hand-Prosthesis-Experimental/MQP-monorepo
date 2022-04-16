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
#include <GMM.h>
#include <Wire.h>
#include "Motor.h"
#include "Haptics.h"
#include "Preferences.h"

BLEServer *pServer = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
long lastNotifyTime = 0;

int mux0readPin = 13;
int mux0Pin0 = 25;
int mux0Pin1 = 26;
int mux0Pin2 = 27;

int mux1readPin = 32;
int mux1Pin0 = 15;
int mux1Pin1 = 14;
int mux1Pin2 = 4;

int potPin = 26;
int currentPin = 27;
volatile int potReading = 0;
volatile int currentReading = 0;
int sharedPotReading = 0;
int sharedCurrentReading = 0;

// Timer setup
#define TIMER0_INTERVAL_US 50

void IRAM_ATTR TimerHandler0();
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t *timer = NULL;

Motor motor(25, 33, &sharedPotReading, &sharedCurrentReading);

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID_TEST1 "1a6a0dc9-7db0-4e5f-8b48-5122af7d0b73"
#define CHARACTERISTIC_UUID_TEST1 "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define SERVICE_UUID_TEST3 "6e400005-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TEST3 "6e400006-b5a3-f393-e0a9-e50e24dcca9e"
#define SERVICE_UUID_VIBCONF "6e400007-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_VIBCONF "6e400008-b5a3-f393-e0a9-e50e24dcca9e"
#define SERVICE_UUID_VELO "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_VELO "6e400004-b5a3-f393-e0a9-e50e24dcca9e"
#define SERVICE_UUID_VELO2 "6e400009-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_VELO2 "6e40000a-b5a3-f393-e0a9-e50e24dcca9e"
#define SERVICE_UUID_VELO2 "6e40000b-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_VELO2 "6e40000d-b5a3-f393-e0a9-e50e24dcca9e"

BLEProp test1(SERVICE_UUID_TEST1, CHARACTERISTIC_UUID_TEST1, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, 4);
BLEProp test3(SERVICE_UUID_TEST3, CHARACTERISTIC_UUID_TEST3, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, 4);
<<<<<<< HEAD
BLEProp velo(SERVICE_UUID_VELO, CHARACTERISTIC_UUID_VELO, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, 4*20);
=======
<<<<<<< HEAD
BLEProp velo1(SERVICE_UUID_VELO1, CHARACTERISTIC_UUID_VELO1, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, 4 * 20);
=======
BLEProp velo(SERVICE_UUID_VELO, CHARACTERISTIC_UUID_VELO, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, 4*20);
>>>>>>> PID with some filtering
>>>>>>> PID with some filtering
BLEProp velo2(SERVICE_UUID_VELO2, CHARACTERISTIC_UUID_VELO2, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, 4);
BLEProp vibConf(SERVICE_UUID_VIBCONF, CHARACTERISTIC_UUID_VIBCONF, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, sizeof(uint8_t) * 15);

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

Preferences preferences;

// Headers
float hallPos(bool debug_prints);
void velostatHandler(bool debug_prints);

void calib_switch();
float adc2v(int adc_val);
long vsRead();
long hRead();
int muxedRead(int mux, int pin);
float muxedReadVolts(int mux, int pin);

long update_elapsed_time();
bool get_vs_calib_button();
bool get_hall_calib_button();

// Velostat Variables
Haptics haptics(2);
const int vs_sen_num = 5;
int veloAddrs[5] = {0, 1, 2, 3, 4};

float veloReadings[vs_sen_num][2] = {
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0}};

int outputStates[vs_sen_num][1] = {
    {0},
    {0},
    {0},
    {0},
    {0}};
uint8_t vibeSettings[vs_sen_num][3];

// TODO increase sample duration
const int vs_time_per_sample = 15;                                   // time per vs sample
const int vs_calib_duration = 10000;                                 // time for vs calibration period. 10 sec
const int total_vs_samples = vs_calib_duration / vs_time_per_sample; // 10000/15 =666

float vs_data[total_vs_samples * vs_sen_num][2] = {0}; //[data sample len][force, force']

// -------------------------------Brian globals
// TODO Fix all of these. All are from arduino nano
// Pins
const int buttonPin = 32;
const int buttonPin2 = 34;
const int calib_button_pin = 35; // D4
// const int potPin = 1; we both used pot pin
int LED_pin = 22; // esp32 onboard

const int Hall_1_Pin = 36; // Changed by drew
const int Hall_2_Pin = 26;
const int Hall_3_Pin = 27;
const int Hall_4_Pin = 15;
const int Hall_5_Pin = 14;
const int Hall_6_Pin = 4;

// TODO refactor old button states
int buttonState = 0;
int buttonState2 = 0;
int calib_button_push = 0;

// States
enum State
{
  standby,
  calibration,
  working
};

State hall_s = standby;
State vs_s = standby;

// Global Consts
const int sensor_num = 6;
const int interval_duration = 2000; // ms
const int intervals = 8;            // 13 is max memorywise
const int samples_per_interval = 125;
const int hall_time_per_sample = interval_duration / samples_per_interval;
const int calib_duration = interval_duration * intervals;
const int total_samples = intervals * samples_per_interval;

// GLobal Vars
int hall_calib_i = 0;
int vs_calib_i = 0;

int hall_1 = 0;
int hall_2 = 0;
int hall_3 = 0;
int hall_4 = 0;
int hall_5 = 0;
int hall_6 = 0;

// Timers
// TODO Refactor these with drews timer
unsigned long start_time;
unsigned long last_time;
unsigned long elapsed_time;

// Kalman Filter
float pos_noise = 0.01;
float noise = .00875;
SimpleKalmanFilter PosKalmanFilter(1, 1, pos_noise);
SimpleKalmanFilter Hall1KalmanFilter(1, 1, noise);
SimpleKalmanFilter Hall2KalmanFilter(1, 1, noise);
SimpleKalmanFilter Hall3KalmanFilter(1, 1, noise);
SimpleKalmanFilter Hall4KalmanFilter(1, 1, noise);
SimpleKalmanFilter Hall5KalmanFilter(1, 1, noise);
SimpleKalmanFilter Hall6KalmanFilter(1, 1, noise);
// KNN Clasifier
KNNClassifier myKNN(sensor_num); // same as sensor_num
// GMM Clasifier
int vibrohaptic_response_num = 3;
Gaussian_Mixture_Model myGMM_diagonal("diagonal", 2, vibrohaptic_response_num);
Gaussian_Mixture_Model myGMM_other("other", 2, vibrohaptic_response_num);

void setup()
{
  Serial.begin(115200);
  preferences.begin("vibro", false);
  if (!preferences.isKey("vibConf"))
  {
    uint8_t initialConfig[5][3] = {
        {0, 47, 51},
        {0, 47, 51},
        {0, 47, 51},
        {0, 47, 51},
        {0, 47, 51}};
    preferences.putBytes("vibConf", initialConfig, sizeof(initialConfig));
  }
  preferences.getBytes("vibConf", vibeSettings, sizeof(vibeSettings));

  // Create the BLE Device
  BLEDevice::init("Prosthesis");
  BLEDevice::setMTU(64); // Wish this worked but I don't think it does on windows
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  test1.attach(pServer);
  test3.attach(pServer);
  vibConf.attach(pServer);
  velo.attach(pServer);
  velo2.attach(pServer);

  Serial.println("Server initialized");

  // Start advertising
  pServer->getAdvertising()->setScanResponse(true);
  pServer->getAdvertising()->start();

  test1.setValue(1.0);
<<<<<<< HEAD
  test3.setValue(0.5);
  velo.setBytes((uint8_t*)veloReadings, 4 * 20);
=======
<<<<<<< HEAD
  test3.setValue(0.0);
  velo1.setBytes((uint8_t *)veloReadings, 4 * 20);
=======
  test3.setValue(0.5);
  velo.setBytes((uint8_t*)veloReadings, 4 * 20);
>>>>>>> PID with some filtering
>>>>>>> PID with some filtering
  velo2.setValue(0.0);
  vibConf.setBytes((uint8_t *)vibeSettings, sizeof(vibeSettings));

  vibConf.notify();
  velo.notify();
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
  int init_i = 0;
  do
  {
    hall_1 = Hall1KalmanFilter.updateEstimate(analogRead(Hall_1_Pin));
    hall_2 = Hall2KalmanFilter.updateEstimate(analogRead(Hall_2_Pin));
    hall_3 = Hall3KalmanFilter.updateEstimate(analogRead(Hall_3_Pin));
    hall_4 = Hall4KalmanFilter.updateEstimate(analogRead(Hall_4_Pin));
    hall_5 = Hall5KalmanFilter.updateEstimate(analogRead(Hall_5_Pin));
    hall_6 = Hall6KalmanFilter.updateEstimate(analogRead(Hall_6_Pin));

    init_i++;
    delay(hall_time_per_sample);
  } while (init_i < 20);

  // Setup timer
  //timer = timerBegin(1, 80, true);
  //timerAttachInterrupt(timer, &TimerHandler0, true);
  //timerAlarmWrite(timer, 500, true);
  // timerAlarmEnable(timer);
}

long lastLoop = 0;

void loop()
{
  // vTaskEnterCritical(&timerMux);
  // sharedPotReading = (int)potReading;
  // sharedCurrentReading = (int)currentReading;
  // vTaskExitCritical(&timerMux);

  sharedCurrentReading = analogRead(currentPin);
  sharedPotReading = analogRead(potPin) + sharedCurrentReading;

  if (micros() > lastLoop + 1000)
  {
    motor.position(test3.getFloat()*(3800-200)+200);
    lastLoop = micros();
  }
  if (deviceConnected)
  {
    if (lastNotifyTime + 10 < millis())
    {
      test1.setValue(sharedPotReading / 4096.0);
      test1.notify();
      velo.setBytes((uint8_t*)veloReadings, 4 * 20);
      velo.notify();
      // velo2.setValue((float)65.7 * powf(analogReadMilliVolts(velopin) / 1000.0, -1.35));
      velo2.notify();
      test3.notify();
      preferences.putBytes("vibConf", vibConf.getData(), vibConf.byteCount);
      vibConf.setBytes((uint8_t *)vibeSettings, sizeof(vibeSettings));
      vibConf.notify();
      lastNotifyTime = millis();
    }
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

    // if ((float)65.7 * powf(analogReadMilliVolts(velo0pin) / 1000.0, -1.35) > 300)
    // {
    //   haptics.vibeselect(0);
    //   haptics.drv->go();
    // }
    // if ((float)65.7 * powf(analogReadMilliVolts(velopin) / 1000.0, -1.35) > 300)
    // {
    //   haptics.vibeselect(1);
    //   haptics.drv->go();
    // }
  }

  int pos = hallPos(false);

}

// Currently Runs at 20khz, see #define TIMER0_INTERVAL_US        50
void IRAM_ATTR TimerHandler0()
{
  portENTER_CRITICAL_ISR(&timerMux);
  currentReading = analogRead(currentPin);
  // Since jank electronics pushes up the gnd for the motor system, the reading
  // would naturally fall by the voltage dropped by the current sense resistor
  potReading = analogRead(potPin) + currentReading;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// Brian helper
float adc2v(int adc_val)
{
  float source_voltage = 4.3;
  int resolution = 4095;
  float voltage = (adc_val * source_voltage) / (resolution);
  return voltage;
}

void hall_calib_switch()
{
  Serial.println("Switching to hall sensor calibration.");
  hall_s = calibration;
  start_time = millis();
  last_time = millis();
  hall_calib_i = 0;

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

float hallPos(bool debug_prints)
{
  switch (hall_s)
  {
  case standby:
    if(debug_prints) Serial.println("standby");
    calib_button_push = get_hall_calib_button();
    if (calib_button_push)
    {
      if (debug_prints)
      {
        Serial.println("PUSHED");
      }
      calib_switch();
    }
    break;

  case calibration:
    // calibrate, storing sum in averaged_data
    if (vs_calib_i < (samples_per_interval * intervals))
    {
      // delay sampling to achive desired rate
      if ((millis() - start_time) < (elapsed_time + hall_time_per_sample))
      {
        if (debug_prints)
        {
          Serial.print("Hall reading delayed");
        }
      }
      else
      {
        bool opening = (int)floor((hall_calib_i) / samples_per_interval) % 2 == 0;
        int pos = 0;
        if (opening)
        {
          pos = hall_calib_i % samples_per_interval;
          digitalWrite(LED_pin, HIGH);
          Serial.print("Calibrating at pos ");
          Serial.print(pos);
          Serial.print("/");
          Serial.print(samples_per_interval);
          Serial.println(" while opening");
        }
        else
        { // count position backwards if closing
          pos = (samples_per_interval - 1) - hall_calib_i % samples_per_interval;
          digitalWrite(LED_pin, LOW);
          Serial.print("Calibrating at pos ");
          Serial.print(pos);
          Serial.print("/");
          Serial.print(samples_per_interval);
          Serial.println(" while closing");
        }

        update_elapsed_time();

        hRead();
        float input[] = {adc2v(hall_1), adc2v(hall_2), adc2v(hall_3), adc2v(hall_4), adc2v(hall_5), adc2v(hall_6)};
        myKNN.addExample(input, pos);

        if (debug_prints) // Print current reading
        {
          Serial.print("Elapsed_time: ");
          Serial.print(elapsed_time);
          Serial.print(". calib_i: ");
          Serial.print(hall_calib_i);
          Serial.print(". opening: ");
          Serial.print(opening);
          Serial.print(". Position: ");
          Serial.println(pos);

          Serial.print("Current reading: ");
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
          Serial.println(hall_6);

          delay(1);
        }

        hall_calib_i++;
      }
    }
    else
    {
      if (debug_prints)
      {
        Serial.println("Making KNN");
      }

      Serial.println("Done calibrating.");
      if (debug_prints)
      {
        Serial.print("myKNN.getCount() = ");
        Serial.println(myKNN.getCount());
        Serial.println();
        // delay(1000);
      }
      hall_s = working; // TODO make motoring
      Serial.print("Calibrated, now returning estimated pos. ");
      digitalWrite(LED_pin, LOW);
      break;
    }
  case working:
    hRead();

    // K Nearest Neighbor
    int k = intervals; // TODO tune
    float input[] = {adc2v(hall_1), adc2v(hall_2), adc2v(hall_3), adc2v(hall_4), adc2v(hall_5), adc2v(hall_6)};
    int position = myKNN.classify(input, k);
    float confidence = myKNN.confidence();
    PosKalmanFilter.setProcessNoise(pos_noise * confidence);
    float estimated_pos = PosKalmanFilter.updateEstimate(position);

    if (debug_prints)
    {
      // Serial.println("Motoring. ");

      Serial.print("Current reading: ");
      Serial.print(input[0], 5);
      Serial.print(", ");
      Serial.print(input[1], 5);
      Serial.print(", ");
      Serial.print(input[2], 5);
      Serial.print(", ");
      Serial.print(input[3], 5);
      Serial.print(", ");
      Serial.print(input[4], 5);
      Serial.print(", ");
      Serial.println(input[5], 5);

      Serial.print("KNN position = ");
      Serial.print(position);

      Serial.print(" Confidence = ");
      Serial.print(confidence);

      Serial.print(" KF estimated_pos = ");
      Serial.print(estimated_pos);
    }
    float target_pos = estimated_pos / samples_per_interval;
    Serial.print(" Confidence: ");
    Serial.print(confidence);
    Serial.print(" Target Position: ");
    Serial.println(target_pos);
    // delay(100);
    return target_pos;

    calib_button_push = get_hall_calib_button();
    if (calib_button_push)
    {
      calib_switch();
    }
    break;
  }
  return 0;
}

void vs_calib_switch()
{
  Serial.println("Switching to velostat sensor calibration.");
  vs_s = calibration;
  start_time = millis();
  last_time = millis();
  vs_calib_i = 0;

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

void velostatHandler(bool debug_prints)
{
  switch (hall_s)
  {
  case standby:
    if (debug_prints)
    {
      Serial.println("In standby.");
    }
    calib_button_push = get_vs_calib_button();
    if (calib_button_push)
    {
      if (debug_prints)
      {
        Serial.println("PUSHED");
      }
      calib_switch();
    }
    break;

  case calibration:
    // calibrate, storing in KNN
    if (vs_calib_i < (samples_per_interval * intervals))
    {
      // delay sampling to achive desired rate
      if ((millis() - start_time) < (elapsed_time + vs_time_per_sample))
      {
        if (debug_prints)
        {
          Serial.print("vs reading delayed");
        }
      }
      else
      { // undelayed
        vsRead();
        for (int i = 0; i < vs_sen_num; i++)
        {
          vs_data[vs_calib_i + i][0] = 0;
          vs_data[vs_calib_i + i][1] = 0;
        }
        vs_calib_i++;
      }
    }
    else
    {
      Serial.print("Calibrated. ");
      if (debug_prints)
      {
        Serial.print("Now updating vs globals. ");
      }
      Serial.println("");

      hall_s = working; // TODO make motoring
      digitalWrite(LED_pin, LOW);
      break;
    }
  case working:

    calib_button_push = get_vs_calib_button();
    if (calib_button_push)
    {
      calib_switch();
    }
    break;
  }
}

bool get_vs_calib_button()
{
  // TODO connect to web UI
  return false;
}

bool get_hall_calib_button()
{
  // TODO connect to web UI
  return false;
}

long vsread()
{

  for (int i = 0; i < sizeof(veloAddrs) / sizeof(int); i++)
  {
    veloReadings[i][1] = (float)(veloReadings[i][0] - veloReadings[i][1]);
    veloReadings[i][0] = (float)65.7 * powf(muxedReadVolts(0, i), -1.35);
  }
  return update_elapsed_time();
}

long hRead()
{
  hall_1 = Hall1KalmanFilter.updateEstimate(analogRead(Hall_1_Pin));
  hall_2 = Hall2KalmanFilter.updateEstimate(analogRead(Hall_2_Pin));
  hall_3 = Hall3KalmanFilter.updateEstimate(analogRead(Hall_3_Pin));
  hall_4 = Hall4KalmanFilter.updateEstimate(analogRead(Hall_4_Pin));
  hall_5 = Hall5KalmanFilter.updateEstimate(analogRead(Hall_5_Pin));
  hall_6 = Hall6KalmanFilter.updateEstimate(analogRead(Hall_6_Pin));
  return update_elapsed_time();
}

long update_elapsed_time()
{
  unsigned long current_time = millis();
  elapsed_time = current_time - start_time;
  return current_time;
}

int muxedRead(int mux, int pin)
{
  int val;
  vTaskEnterCritical(&timerMux);
  if (mux == 0)
  {
    digitalWrite(mux0Pin2, pin >> 2 & 1);
    digitalWrite(mux0Pin1, pin >> 1 & 1);
    digitalWrite(mux0Pin0, pin & 1);
    val = analogRead(mux0readPin);
  }
  else if (mux == 1)
  {
    digitalWrite(mux0Pin2, pin >> 2 & 1);
    digitalWrite(mux0Pin1, pin >> 1 & 1);
    digitalWrite(mux0Pin0, pin & 1);
    val = analogRead(mux1readPin);
  }
  vTaskExitCritical(&timerMux);
  return val;
}

float muxedReadVolts(int mux, int pin)
{
  int val;
  vTaskEnterCritical(&timerMux);
  if (mux == 0)
  {
    digitalWrite(mux0Pin2, pin >> 2 & 1);
    digitalWrite(mux0Pin1, pin >> 1 & 1);
    digitalWrite(mux0Pin0, pin & 1);
    val = analogReadMilliVolts(mux0readPin);
  }
  else if (mux == 1)
  {
    digitalWrite(mux0Pin2, pin >> 2 & 1);
    digitalWrite(mux0Pin1, pin >> 1 & 1);
    digitalWrite(mux0Pin0, pin & 1);
    val = analogReadMilliVolts(mux1readPin);
  }
  vTaskExitCritical(&timerMux);
  return val / 1000.0;
}