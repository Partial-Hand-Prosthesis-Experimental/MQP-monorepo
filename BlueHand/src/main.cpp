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

// Added to try to fix map issue
// #include <iostream>
// #include <iterator>
// #include <map>

BLEServer *pServer = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
long lastNotifyTime = 0;

int muxreadPin = 38;
int muxPin0 = 25;
int muxPin1 = 26;
int muxPin2 = 27;

int potPin = 6;
int currentPin = 7;

// volatile int potReading = 0;
// volatile int currentReading = 0;
int sharedPotReading = 0;
int sharedCurrentReading = 0;

// Timer setup
#define TIMER0_INTERVAL_US 50

void IRAM_ATTR TimerHandler0();
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t *timer = NULL;

Motor motor(18, 19, &sharedPotReading, &sharedCurrentReading);

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
#define SERVICE_UUID_MODES "6e40000b-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_MODES "6e40000d-b5a3-f393-e0a9-e50e24dcca9e"

BLEProp test1(SERVICE_UUID_TEST1, CHARACTERISTIC_UUID_TEST1, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, 4);
BLEProp test3(SERVICE_UUID_TEST3, CHARACTERISTIC_UUID_TEST3, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, 4);

// 4 bytes auto casted to float32 on web, use != mod 4
BLEProp modes(SERVICE_UUID_MODES, CHARACTERISTIC_UUID_MODES, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, 5);
uint8_t currentModes[5] = {0, 0, 0, 0, 0};
uint8_t lastModes[5] = {0, 0, 0, 0, 0};
BLEProp velo(SERVICE_UUID_VELO, CHARACTERISTIC_UUID_VELO, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, 4 * 20);

BLEProp velo2(SERVICE_UUID_VELO2, CHARACTERISTIC_UUID_VELO2, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, 4);

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
void velostatHandler(bool debug_prints, bool diagonal);

void hall_calib_switch();
void vs_calib_switch();
float adc2v(int adc_val);
long vsRead();
long hRead();
void hapticHandler(bool debug_prints);
int muxedRead(int pin);
float muxedReadVolts(int pin);

long update_elapsed_time();
long update_hall_elapsed_time();
bool get_vs_calib_button();
bool get_hall_calib_button();

// Velostat Variables
Haptics haptics(5);
const int vs_sen_num = 5;
int veloAddrs[vs_sen_num] = {32, 33, 39, 37, 36};

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

uint8_t vibeSettings[vs_sen_num][3][3];

BLEProp vibConf(SERVICE_UUID_VIBCONF, CHARACTERISTIC_UUID_VIBCONF, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE, sizeof(vibeSettings));

// TODO increase sample duration
const int vs_time_per_sample = 20;   // time per vs sample
const int vs_calib_duration = 10000; // time for vs calibration period. in millisec
const int vs_sample_num = vs_calib_duration / vs_time_per_sample;
const int total_vs_data = vs_sample_num * vs_sen_num; // 4000/40 * 5 =500
const int test = 2;
double **vs_data;

// -------------------------------Brian globals
// TODO Fix all of these. All are from arduino nano
// Pins
const int buttonPin = 32;
const int buttonPin2 = 34;
const int calib_button_pin = 35; // D4
// const int potPin = 1; we both used pot pin

const int Hall_1_Pin = 0; // Changed by drew
const int Hall_2_Pin = 1;
const int Hall_3_Pin = 2;
const int Hall_4_Pin = 3;
const int Hall_5_Pin = 4;
const int Hall_6_Pin = 5;

// TODO refactor old button states
int buttonState = 0;
int buttonState2 = 0;
bool hall_calib_button_push = 0;
bool vs_calib_button_push = 0;

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
const int interval_duration = 6000;                                        // ms
const int intervals = 10;                                                  // 13 is max memorywise
const int samples_per_interval = 125;                                      // 110mm arc len on thumb tip / 1mm percision goal
const int hall_time_per_sample = interval_duration / samples_per_interval; // 2000/110 = 18.2s
const int calib_duration = interval_duration * intervals;
const int total_hall_samples = intervals * samples_per_interval; // --------------------------------make smaller to fix memory issues

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
long vs_start_time;
long hall_sample_time;
long hall_elapsed_time;
long elapsed_time;
long vs_old_start_time;
long hall_old_start_time;
int sample_rate_threshold = 10; // ms

// Kalman Filter
float pos_noise = 0.01;
float hall_noise = .00875;
float vs_noise = .02;
SimpleKalmanFilter PosKalmanFilter(1, 1, pos_noise);
SimpleKalmanFilter Hall1KalmanFilter(1, 1, hall_noise);
SimpleKalmanFilter Hall2KalmanFilter(1, 1, hall_noise);
SimpleKalmanFilter Hall3KalmanFilter(1, 1, hall_noise);
SimpleKalmanFilter Hall4KalmanFilter(1, 1, hall_noise);
SimpleKalmanFilter Hall5KalmanFilter(1, 1, hall_noise);
SimpleKalmanFilter Hall6KalmanFilter(1, 1, hall_noise);

SimpleKalmanFilter vs1KalmanFilter(1, 1, vs_noise);
SimpleKalmanFilter vs2KalmanFilter(1, 1, vs_noise);
SimpleKalmanFilter vs3KalmanFilter(1, 1, vs_noise);
SimpleKalmanFilter vs4KalmanFilter(1, 1, vs_noise);
SimpleKalmanFilter vs5KalmanFilter(1, 1, vs_noise);

SimpleKalmanFilter vs_kalmans[vs_sen_num] = {vs1KalmanFilter, vs2KalmanFilter, vs3KalmanFilter, vs4KalmanFilter, vs5KalmanFilter};

// KNN Clasifier
KNNClassifier myKNN(sensor_num); // same as sensor_num
// GMM Clasifier
int vibrohaptic_response_num = 3;
// KMeans myKMeans(2, vibrohaptic_response_num);
Gaussian_Mixture_Model clusterer("other", 2, vibrohaptic_response_num);

// print booleans
bool print_tactile_data = false;
bool print_hall_data = false;
bool print_torque_data = false;
bool print_pos_max_force_data = false;

void setup()
{

  Serial.begin(115200);

  pinMode(muxPin0, OUTPUT);
  pinMode(muxPin1, OUTPUT);
  pinMode(muxPin2, OUTPUT);
  pinMode(muxreadPin, INPUT);

  preferences.begin("vibro", false);
  if (!preferences.isKey("vibConf"))
  {
    if (!print_tactile_data && !print_tactile_data && !print_torque_data && !print_pos_max_force_data)
    {
      Serial.println("No vibConf key found. Creating...");
    }
    uint8_t initialConfig[vs_sen_num][3][3];
    for (int i = 0; i < vs_sen_num; i++)
    {
      initialConfig[i][0][0] = 0;
      initialConfig[i][0][1] = 0;
      initialConfig[i][0][2] = 0;

      initialConfig[i][1][0] = 0;
      initialConfig[i][1][1] = 47;
      initialConfig[i][1][2] = 48;

      initialConfig[i][2][0] = 0;
      initialConfig[i][2][1] = 50;
      initialConfig[i][2][2] = 51;
    }
    if (!print_tactile_data && !print_tactile_data && !print_torque_data && !print_pos_max_force_data)
    {
      Serial.print("Generated ");
      Serial.print(sizeof(initialConfig));
      Serial.println(" bytes");
    }
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

  if (!preferences.isKey("modes"))
  {
    uint8_t defaultModes[5] = {4, 4, 2, 4, 1};
    preferences.putBytes("modes", defaultModes, sizeof(defaultModes));
  }
  preferences.getBytes("modes", currentModes, sizeof(currentModes));
  modes.attach(pServer);
  if (!print_tactile_data && !print_tactile_data && !print_torque_data && !print_pos_max_force_data)
  {
    Serial.println("Server initialized");
  }
  // Start advertising
  pServer->getAdvertising()->setScanResponse(true);
  pServer->getAdvertising()->start();

  test1.setValue(1.0);

  test3.setValue(0.2); // Start with low current limit to prevent windup during calibration
  velo.setBytes((uint8_t *)veloReadings, 4 * 20);

  velo2.setValue(0.0);
  vibConf.setBytes((uint8_t *)vibeSettings, sizeof(vibeSettings));

  modes.setBytes(currentModes, 5);

  vibConf.notify();
  velo.notify();
  velo2.notify();
  test1.notify();
  test3.notify();
  if (!print_tactile_data && !print_tactile_data && !print_torque_data && !print_pos_max_force_data)
  {
    Serial.println("Waiting a client connection to notify...");
  }
  // Brian Setup
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(calib_button_pin, INPUT);

  // Kalman Filter Setup
  int init_i = 0;
  do
  {
    hRead();
    vsRead();
    init_i++;
    delay(hall_time_per_sample);
  } while (init_i < 20);

  // Setup timer
  // timer = timerBegin(1, 80, true);
  // timerAttachInterrupt(timer, &TimerHandler0, true);
  // timerAlarmWrite(timer, 500, true);
  // timerAlarmEnable(timer);
}

long lastLoop = 0;

void loop()
{
  static bool debug = true;
  sharedCurrentReading = muxedRead(currentPin);
  if(print_torque_data){
    Serial.println(sharedCurrentReading);
  }
  sharedPotReading = muxedRead(potPin) + sharedCurrentReading;

  float pos = hallPos(debug);

  if (micros() > lastLoop + 1000)
  {
    static float velocity_target = 0.0;
    const float velocity_rate = 0.02;
    float target;
    switch (currentModes[0])
    {       // Motor Mode indicator
    case 0: // Off
      velocity_target = 0.0;
      motor.speed(0.0);
      break;
    case 1: // Position
      motor.position(pos * (3800 - 350) + 350, constrain(test3.getFloat() * 0.2, 0, 0.2));
      break;
    case 2: // Velocity
      velocity_target = constrain(velocity_target + (pos * 2 - 1) * velocity_rate, 0, 1);
      motor.position(velocity_target * (3800 - 350) + 350, constrain(test3.getFloat() * 0.2, 0, 0.2));
      break;
    case 3: // Velocity + Torque
      velocity_target = constrain(velocity_target + (pos * 2 - 1) * velocity_rate, 0, 1);
      motor.position(velocity_target * (3800 - 350) + 350, constrain((pos * 2 - 1) * 0.2, 0, 0.2));
      break;
    case 4: // Sweep
      target = 0.5 * (sin(2 * PI * millis() / (10000.0)) + 1);
      motor.position(target * (3800 - 350) + 350, constrain(test3.getFloat() * 0.2, 0, 0.2));
      break;
    case 5:
      motor.position(350, constrain(test3.getFloat() * 0.2, 0, 0.2));
      break;
    case 6:
      motor.position(3800, constrain(test3.getFloat() * 0.2, 0, 0.2));
      break;
    }
    lastLoop = micros();
  }

  if (deviceConnected)
  {
    if (lastNotifyTime + 10 < millis())
    {
      // test1.setValue(pos+abs(0.75*sin(millis()/1000.0)));
      test1.setValue(sharedPotReading);
      test1.notify();

      velo.setBytes((uint8_t *)veloReadings, 4 * 20);
      velo.notify();
      // velo2.setValue((float)65.7 * powf(analogReadMilliVolts(velopin) / 1000.0, -1.35));
      velo2.notify();
      test3.notify();
      preferences.putBytes("vibConf", vibConf.getData(), vibConf.byteCount);
      preferences.getBytes("vibConf", vibeSettings, sizeof(vibeSettings));
      vibConf.setBytes((uint8_t *)vibeSettings, sizeof(vibeSettings));
      vibConf.notify();
      uint8_t newModes[5];
      for (int i = 0; i < 5; i++)
      {
        newModes[i] = modes.getData()[i];
      }
      // Set Vibration Mode Permanently anytime its switched
      if (newModes[2] != currentModes[2])
      {
        currentModes[2] = newModes[2];
        preferences.putBytes("modes", currentModes, sizeof(currentModes));
      }
      currentModes[0] = newModes[0];
      currentModes[0] = newModes[0];
      currentModes[2] = newModes[2];
      currentModes[1] = newModes[1];
      currentModes[3] = newModes[3];
      switch (newModes[4])
      {
      case 0: // Off
        debug = false;
        break;
      case 1: // debug
        debug = true;
        break;
      case 2: // Save defaults
        preferences.putBytes("modes", currentModes, sizeof(currentModes));
      case 3: // Load defaults
        preferences.getBytes("modes", currentModes, sizeof(currentModes));
      }
      modes.setBytes(currentModes, 5);
      modes.notify();

      lastNotifyTime = millis();
    }
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    motor.speed(0.0);
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->disconnect(0);
    pServer->getAdvertising()->setScanResponse(true);
    pServer->startAdvertising(); // restart advertising
    if (!print_tactile_data && !print_tactile_data && !print_torque_data && !print_pos_max_force_data)
    {
      Serial.println("start advertising");
    }
    oldDeviceConnected = deviceConnected;
    if (!print_tactile_data && !print_tactile_data && !print_torque_data && !print_pos_max_force_data)
    {
      Serial.println("Client Disconnected");
    }
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    if (!print_tactile_data && !print_tactile_data && !print_torque_data && !print_pos_max_force_data)
    {
      Serial.println("Client Connected");
    }
  }

  velostatHandler(false, false);
  hapticHandler(false);

  // delay(1);
}

// Currently Runs at 20khz, see #define TIMER0_INTERVAL_US        50
void IRAM_ATTR TimerHandler0()
{
  portENTER_CRITICAL_ISR(&timerMux);
  // currentReading = analogRead(currentPin);
  // Since jank electronics pushes up the gnd for the motor system, the reading
  // would naturally fall by the voltage dropped by the current sense resistor
  // potReading = analogRead(potPin) + currentReading;
  portEXIT_CRITICAL_ISR(&timerMux);
}

float adc2v(int adc_val)
{
  float source_voltage = 4.3;
  int resolution = 4095;
  float voltage = (adc_val * source_voltage) / (resolution);
  return voltage;
}

void hall_calib_switch()
{
  // Serial.println("Switching to hall sensor calibration.");
  hall_s = calibration;
  hall_sample_time = millis();
  hall_calib_i = 0;
}

float hallPos(bool debug_prints)
{
  switch (hall_s)
  {
  case standby:

    // calib_button_push = get_hall_calib_button();
    hall_calib_button_push = (vs_s == working);
    if (hall_calib_button_push && vs_s != calibration)
    {

      hall_calib_switch();
    }
    break;

  case calibration:
    // calibrate, storing sum in averaged_data
    if (hall_calib_i < (total_hall_samples))
    {
      // delay sampling to achive desired rate
      if ((millis() - hall_old_start_time) < (hall_time_per_sample))
      {
      }
      else
      {
        hall_sample_time = hRead();
        // if (debug_prints)
        // {

        //   Serial.print(" actual interval: ");
        //   Serial.print(hall_sample_time - hall_old_start_time);
        //   Serial.print(" desired interval: ");
        //   Serial.println(hall_time_per_sample);
        // }
        int diff = hall_sample_time - hall_old_start_time;
        // if (diff > sample_rate_threshold)
        // {
        //   Serial.print("Large sample rate difference detected. Diference of ");
        //   Serial.println(diff);
        // }

        bool opening = (int)floor((hall_calib_i) / samples_per_interval) % 2 == 0;
        int pos = 0;
        if (opening)
        {
          pos = hall_calib_i % samples_per_interval;
          if (debug_prints)
          {
            Serial.print("Calibrating at pos ");
            Serial.print(pos);
            Serial.print("/");
            Serial.print(samples_per_interval);
            Serial.println(" while opening");
          }
        }
        else
        { // count position backwards if closing
          pos = (samples_per_interval - 1) - hall_calib_i % samples_per_interval;

          if (debug_prints)
          {
            Serial.print("Calibrating at pos ");
            Serial.print(pos);
            Serial.print("/");
            Serial.print(samples_per_interval);
            Serial.println(" while closing");
          }
        }

        float input[] = {adc2v(hall_1), adc2v(hall_2), adc2v(hall_3), adc2v(hall_4), adc2v(hall_5), adc2v(hall_6)};
        myKNN.addExample(input, pos);
        hall_old_start_time = hall_sample_time;

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

          // delay(1);
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
    }

    break;

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
    if (debug_prints)
    {
      Serial.print(" Confidence: ");
      Serial.print(confidence);
      Serial.print(" Target Position: ");
      Serial.println(target_pos);
    }
    // delay(100);

    hall_calib_button_push = get_hall_calib_button();
    if (hall_calib_button_push)
    {
      hall_calib_switch();
    }
    return target_pos;
    break;
  }
  return 0;
}

void vs_calib_switch()
{
  // Serial.println("Switching to velostat sensor calibration.");
  vs_s = calibration;
  vs_start_time = millis();
  vs_old_start_time = vs_start_time;
  vs_calib_i = 0;

  vs_data = (double **)malloc(total_vs_data * sizeof(double *));
  for (int i = 0; i < total_vs_data; i++)
    vs_data[i] = (double *)malloc(2 * sizeof(double));
}

void velostatHandler(bool debug_prints, bool diagonal)
{
  switch (vs_s)
  {
  case standby:
    if (debug_prints)
    {
      Serial.println("VS in standby.");
    }
    // calib_button_push = get_vs_calib_button();
    vs_calib_button_push = true;
    if (vs_calib_button_push && vs_s != calibration)
    {
      if (debug_prints)
      {
        Serial.println("PUSHED");
      }
      vs_calib_switch();
    }
    break;

  case calibration:
    // calibrate, storing in KNN
    if (debug_prints || print_tactile_data)
    {
      Serial.print("Calibrating velostat clasifier");
      Serial.print(vs_calib_i);
      Serial.print("/");
      Serial.println(vs_sample_num);
    }
    if (vs_calib_i < (vs_sample_num))

    {
      // delay sampling to achive desired rate
      if ((millis() - (vs_old_start_time)) < (vs_time_per_sample))
      {
        if (debug_prints)
        {
          // Serial.print("vs reading delayed");
        }
      }
      else
      { // undelayed

        vs_start_time = vsRead();

        if (debug_prints)
        {
          Serial.print(" actual interval: ");
          Serial.print(vs_start_time - (vs_old_start_time));
          Serial.print(" desired interval: ");
          Serial.println(vs_time_per_sample);
        }

        for (int i = 0; i < vs_sen_num; i++)
        {
          vs_data[(vs_calib_i * 5) + i][0] = veloReadings[i][0];
          vs_data[(vs_calib_i * 5) + i][1] = veloReadings[i][1];
          if (debug_prints)
          {
            Serial.print(veloReadings[i][0]);
            Serial.print(", ");
            Serial.println(veloReadings[i][1]);
          }
        }
        vs_calib_i++;
        vs_old_start_time = vs_start_time;
      }
    }
    else
    {
      if (!print_tactile_data && !print_tactile_data && !print_torque_data && !print_pos_max_force_data)
      {
        Serial.println("Calibrated, initialising Kmeans...");
        Serial.println(vs_data[1][1]);
      }
      clusterer.Initialize(total_vs_data, vs_data);
      if (!print_tactile_data && !print_tactile_data && !print_torque_data && !print_pos_max_force_data)
      {
        Serial.println("Kmeans initialized, now clasifying VS inputs. ");
      }
      for (int i = 0; i < total_vs_data; i++)
        free(vs_data[i]);
      free(vs_data);

      vs_s = working; // TODO make motoring
    }
    break;
  case working:
    vsRead();
    for (int i = 0; i < vs_sen_num; i++)
    {
      double d[] = {(double)veloReadings[i][0], (double)veloReadings[i][1]};

      int classification = clusterer.Classify(d);

      outputStates[i][0] = classification;
      if (debug_prints)
      {
        Serial.print("Velostat reeding at segment ");
        Serial.print(i);
        Serial.print(" : ");
        Serial.print(veloReadings[i][0]);
        Serial.print(", ");
        Serial.println(veloReadings[i][1]);
      }
      // delay(5000);
    }

    if (print_tactile_data)
    {
      for (int i = 0; i < vs_sen_num; i++)
      {
        Serial.print(veloReadings[i][0]);
        Serial.print(", ");
        Serial.print(veloReadings[i][1]);
        Serial.print(", ");
        Serial.print(outputStates[i][0]);
        Serial.println(", ");
      }
    }

    if (debug_prints)
    {
      Serial.print("VS tactile clasifications: ");
      for (int i = 0; i < vs_sen_num; i++)
      {
        Serial.print(outputStates[i][0]);
        Serial.print(", ");
      }
      Serial.println("");

      Serial.print("Clasifications likklyhood: ");
      for (int i = 0; i < vs_sen_num; i++)
      {
        double d[] = {(double)veloReadings[i][0], (double)veloReadings[i][1]};
        Serial.print(clusterer.Calculate_Likelihood(d));
        Serial.print(", ");
      }
      Serial.println("");
    }

    vs_calib_button_push = get_vs_calib_button();
    if (vs_calib_button_push)
    {
      vs_calib_switch();
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

long vsRead()
{
  long time = update_elapsed_time();
  float lastVelo;
  for (int i = 0; i < sizeof(veloAddrs) / sizeof(int); i++)
  {
    lastVelo = veloReadings[i][0];
    veloReadings[i][0] = vs_kalmans[i].updateEstimate((float)65.7 * powf(analogReadMilliVolts(veloAddrs[i]) / 1000.0, -1.35)); // V to g
    veloReadings[i][1] = (float)(veloReadings[i][0] - lastVelo);
  }
  return time;
}
long hRead()
{
  long time = update_hall_elapsed_time();
  hall_1 = Hall1KalmanFilter.updateEstimate(muxedRead(Hall_1_Pin));
  hall_2 = Hall2KalmanFilter.updateEstimate(muxedRead(Hall_2_Pin));
  hall_3 = Hall3KalmanFilter.updateEstimate(muxedRead(Hall_3_Pin));
  hall_4 = Hall4KalmanFilter.updateEstimate(muxedRead(Hall_4_Pin));
  hall_5 = Hall5KalmanFilter.updateEstimate(muxedRead(Hall_5_Pin));
  hall_6 = Hall6KalmanFilter.updateEstimate(muxedRead(Hall_6_Pin));
  return time;
}

void hapticHandler(bool debug_prints)
{
  for (int i = 0; i < 5; i++)
  {
    haptics.vibeselect(i);
    // TODO: change second index to current haptic mode from UI
    haptics.drv->setWaveform(0, vibeSettings[i][1][outputStates[i][0]]);
    haptics.drv->go();
  }
}

long update_elapsed_time()
{
  long current_time = millis();
  elapsed_time = current_time - vs_start_time;
  return current_time;
}
long update_hall_elapsed_time()
{
  long current_time = millis();
  hall_elapsed_time = current_time - hall_sample_time;
  return current_time;
}

int muxedRead(int pin)
{
  int val;
  // vTaskEnterCritical(&timerMux);
  digitalWrite(muxPin2, pin >> 2 & 1);
  digitalWrite(muxPin1, pin >> 1 & 1);
  digitalWrite(muxPin0, pin & 1);
  val = analogRead(muxreadPin);
  // vTaskExitCritical(&timerMux);
  return val;
}

float muxedReadVolts(int pin)
{
  int val;
  // vTaskEnterCritical(&timerMux);
  digitalWrite(muxPin2, pin >> 2 & 1);
  digitalWrite(muxPin1, pin >> 1 & 1);
  digitalWrite(muxPin0, pin & 1);
  val = analogReadMilliVolts(muxreadPin);
  // vTaskExitCritical(&timerMux);

  return val / 1000.0;
}