#include "Arduino.h"
#include "SensorModule.h"

SensorModule::SensorModule(int sensorPin1, int sensorPin2)
{
  _sensorPin1 = sensorPin1;
  _sensorPin2 = sensorPin2;
}

int SensorModule::ReadFirst() {
  
}

int SensorModule::ReadSecond() {
  
}
