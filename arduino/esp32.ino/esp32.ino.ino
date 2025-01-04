// GPIO list: https://i.ibb.co/BKn4dgL/ESP32-DEV-CP2102-C-003.jpg

#include <Arduino.h>
#include <Encoder.h>
#include <Wire.h>

// ----------------------------------------------------------------------------------- Pin Configurations
Encoder encoder_left_front(36, 39);
Encoder encoder_left_back(34, 35);
Encoder encoder_right_front(32, 33);
Encoder encoder_right_back(25, 26);

const int motorSpeedLeftPin = 27; // PWM-capable pin
const int motorSpeedRightPin = 14; // PWM-capable pin
const int motorDirectionLeftAPin = 12;
const int motorDirectionLeftBPin = 13;
const int motorDirectionRightAPin = 23;
const int motorDirectionRightBPin = 21;
// ----------------------------------------------------------------------------------- Settings
const int frequency = 30000; // PWM frequency (30 kHz) 
const int resolution = 8; // Must be changed in "updateSpeedController" if this is modified

void setup() 
{
  Serial.begin(9600);
  Wire.begin(0x20); // Setup i2c slave using pin 21 (SCL) and 22(SDA) and #2 address
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event

  // Configure PWM functionality for each channel
  ledcAttach(motorSpeedLeftPin, frequency, resolution);
  ledcAttach(motorSpeedRightPin, frequency, resolution);

  // Setup the motor direction pins
  pinMode(motorDirectionLeftAPin, OUTPUT);
  pinMode(motorDirectionLeftBPin, OUTPUT);
  pinMode(motorDirectionRightAPin, OUTPUT);
  pinMode(motorDirectionRightBPin, OUTPUT);
}

long distanceLeft = 0;
long distanceRight = 0;
float speedRequestLeft = 0;
float speedRequestRight = 0;
float currentSpeedLeft = -999;
float currentSpeedRight = -999;

void loop() 
{
  Serial.println("Hello Computer");

  delay(10);

  // Get the average distance traveled on each side
  distanceLeft += ( encoder_left_back.readAndReset() + encoder_left_front.readAndReset() ) / 2;
  distanceRight += ( encoder_right_back.readAndReset() + encoder_right_front.readAndReset() ) / 2;

  if(speedRequestLeft != currentSpeedLeft)
  {
    currentSpeedLeft = speedRequestLeft;
    updateSpeedController(motorSpeedLeftPin, motorDirectionLeftAPin, motorDirectionLeftBPin, currentSpeedLeft);
  }

  if(speedRequestRight != currentSpeedRight)
  {
    currentSpeedRight = speedRequestRight;
    updateSpeedController(motorSpeedRightPin, motorDirectionRightAPin, motorDirectionRightBPin, currentSpeedRight);
  }
}

void updateSpeedController(int pin, int a, int b, float speed)
{
  // Write the new speed. Since we configured the channel with 8bits, the % has to be changed to a 0-255 scale
  // We should check ledcFade to smooth out the transitions, specially on start/stop
  ledcWrite(pin, round(abs(speed) * 255.0 / 100.0));

  // Write the direction
  if(speed > 0)
  {
    digitalWrite(a, HIGH);
    digitalWrite(b, LOW);
  }
  else if(speed < 0)
  {
    digitalWrite(a, LOW);
    digitalWrite(b, HIGH);
  }
  else
  {
    digitalWrite(a, LOW);
    digitalWrite(b, LOW);
  }
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() 
{
  long distances[2] = {distanceLeft, distanceRight};
  distanceLeft = distanceRight = 0;

  Wire.write((unsigned char*)&distances, sizeof(long) * 2);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  if(howMany != 8) // 2 float
    return; // shouldn't happen
  
  unsigned char buff[8];
  int read = 0;
  while(Wire.available()) // loop through all but the last
  {
    *(buff + read) = Wire.read(); // receive 1 byte;
    ++read;
  }
  
  if(read != howMany)
    return; // shouldn't happen

  speedRequestLeft = ((float*)buff)[0];
  speedRequestRight = ((float*)buff)[1];
}
