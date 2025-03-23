#ifndef Functions_h
#define Functions_h
#include "Arduino.h"

class Functions
{
  public:
    Functions(int sensorPin1, int sensorPin2, int sensorIndicatorPin);
    void Functions::Update(unsigned long currentTime);
    bool Functions::IsOncePerSecondEvent();
    void Functions::SetPWMOnPin9(uint16_t value);
    void Functions::SetPWMOnPin10(uint16_t value);
    bool ReadFirstSensor();
    bool ReadSecondSensor();
    void Functions::SetSensorIndicatorPin(bool active);
  private:
    int _sensorPin1;
    int _sensorPin2;
    int _sensorIndicatorPin;
};

#endif
