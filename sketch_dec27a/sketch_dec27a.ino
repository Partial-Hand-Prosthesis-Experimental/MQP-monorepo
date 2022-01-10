#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#include <Servo.h>

Servo myservo; // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0; // variable to store the servo position
const int buttonPin = 2;
const int buttonPin2 = 3;
int potPin = 1;
int buttonState = 0;
int buttonState2 = 0;
int val = 0;
int batterylevel = 0;
void setup()
{
  myservo.attach(9); // attaches the servo on pin 9 to the servo object
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin2, INPUT);

  Serial.begin(115200);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  delay(100);
  display.clearDisplay();

  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(20, 16);
  // Display static text
  display.println("Hello"); //Write custom text here
  display.setCursor(12, 40);
  display.println("Payton");
  display.display();
  delay(1000);
  display.clearDisplay();
}

void loop()
{
  val = analogRead(potPin);
  val = map(val, 0, 1023, 0, 5);
  buttonState = digitalRead(buttonPin);
  buttonState2 = digitalRead(buttonPin2);
  display.drawTriangle(100, 40, 120, 40, 110, 54, WHITE);
  display.drawTriangle(100, 30, 120, 30, 110, 16, WHITE);

  if (buttonState == HIGH) 
  { // goes from 0 degrees to 180 degrees
    
    // in steps of 1 degree
    pos = pos - val; // MOVED to before servo angle constrain
    
    if (pos > 180)
    {
      pos = 180;
    }
    if (pos < 0)
    {
      pos = 0;
    }
    myservo.write(pos); // tell servo to go to position in variable 'pos'

    display.fillTriangle(100, 30, 120, 30, 110, 16, WHITE);

    delay(15); // waits 15 ms for the servo to reach the position
  }

  if (buttonState2 == HIGH)
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    pos = pos + val; // MOVED to before servo angle constrain
    if (pos > 180)
    {
      pos = 180;
    }
    if (pos < 0)
    {
      pos = 0;
    }
    myservo.write(pos); // tell servo to go to position in variable 'pos'
    display.fillTriangle(100, 40, 120, 40, 110, 54, WHITE);
    delay(15); // waits 15 ms for the servo to reach the position
  }
  display.setTextSize(1);      //text size
  display.setTextColor(WHITE); //text color
  display.setCursor(0, 0);
  display.println("Paytons magic hand"); //display out text 1st line
  display.setTextColor(BLACK, WHITE);
  display.println();
  int sensorValue = analogRead(A0);             // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = sensorValue * (5.0 / 1023.0); // print out the value you read:
  display.println(voltage);
  // batterylevel = map(voltage, 1, 2.5, 0, 60); // Dead code, commented out
  batterylevel = (int)(voltage * 16.21);
  display.fillRect(0, 30, batterylevel, 20, WHITE);

  display.setTextSize(1);
  display.setTextColor(WHITE); //text color
  display.println();
  display.setTextSize(2); //text size
  display.drawRect(0, 30, 60, 20, WHITE);
  display.fillRect(60, 34, 4, 12, WHITE);

  display.setCursor(75, 30);
  display.print("P");
  display.print(val);
  display.display();
  delay(1); //delay time 1 mili second before display.clearDisplay(); MOVED from before text and batt display additions
  display.clearDisplay();
}
