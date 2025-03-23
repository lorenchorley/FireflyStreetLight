#include "Arduino.h"
#include "Functions.h"

Functions::Functions(int sensorPin1, int sensorPin2, int sensorIndicatorPin)
{
  _sensorPin1 = sensorPin1;
  _sensorPin2 = sensorPin2;
  _sensorIndicatorPin = sensorIndicatorPin;
}

bool OncePerSecondEvent = false;
unsigned long previousWholeSecondsFigure = 0;

void Functions::Update(unsigned long currentTime) {
  unsigned long wholeSecondsFigure = currentTime / 1000;
  OncePerSecondEvent = previousWholeSecondsFigure != wholeSecondsFigure;
  previousWholeSecondsFigure = wholeSecondsFigure; 
}

bool Functions::IsOncePerSecondEvent() {
  return OncePerSecondEvent;
}

void Functions::SetPWMOnPin9(uint16_t value) {
  // Ensure value is within 0-4095
  if (value > 4095) {
    value = 4095;
  } else if (value < 0) {
    value = 0;
  }
  
  // Set the Output Compare Register for Timer 1
  OCR1A = value; // Set the PWM value for pin 9
}

void Functions::SetPWMOnPin10(uint16_t value) {
  // Ensure value is within 0-4095
  if (value > 4095) {
    value = 4095;
  } else if (value < 0) {
    value = 0;
  }
  
  // Set the Output Compare Register for Timer 1
  OCR1B = value; // Set the PWM value for pin 10
}

bool Functions::ReadFirstSensor() {
  return digitalRead(_sensorPin1) == HIGH ? true : false;
}

bool Functions::ReadSecondSensor() {
  return digitalRead(_sensorPin2) == HIGH ? true : false;
}

void Functions::SetSensorIndicatorPin(bool active) {
  digitalWrite(_sensorIndicatorPin, active ? HIGH : LOW);
}
