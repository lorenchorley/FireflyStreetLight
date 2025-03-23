#ifndef SensorModuleV1_h
#define SensorModuleV1_h
#include "Arduino.h"

class SensorModule
{
  public:
    SensorModule(int sensorPin1, int sensorPin2);
    int ReadFirst();
    int ReadSecond();
  private:
    int _sensorPin1;
    int _sensorPin2;
};

#endif
