#include <Wire.h>
#include "Adafruit_DRV2605.h"
 
Adafruit_DRV2605 drv;
int VelostatPin = 15;
 
void setup() {
  Serial.begin(9600);
  Serial.println("DRV test");
  drv.begin();

  // I2C trigger by sending 'go' command
  drv.setMode(DRV2605_MODE_INTTRIG); // default, internal trigger when sending GO command
 
  drv.selectLibrary(1);
  drv.setWaveform(0, 14);  // ramp up medium 1, see datasheet part 11.2
  drv.setWaveform(1, 47);  // strong click 100%, see datasheet part 11.2
  drv.setWaveform(2, 0);  // end of waveforms
}
 
void loop() {
  int RawSensorValue = analogRead(VelostatPin);
  double Value = adc2voltage(RawSensorValue);
  Serial.println(Value);
  if(Value < 1)
    {
      Serial.println("go");
      drv.go();
    }
    
}

double adc2voltage(int ADCread) {
  float source_voltage = 5;
  int resolution = 4096;
  double voltage = (ADCread * source_voltage ) / (resolution);
  return voltage;

}
