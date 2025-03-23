#ifndef BehaviourModuleV1_h
#define BehaviourModuleV1_h
#include "Arduino.h"
#include "SensorModule.h"
#include "Functions.h"

class BehaviourModuleV1
{
  public:
    BehaviourModuleV1(SensorModule& sensors, Functions& f, int sensorIndicator);
    void SetInitialValues();
    void Tick(unsigned long currentTime, float deltaT);
  private:
    void BehaviourModuleV1::ReadSensorValues(unsigned long currentTime);
    void BehaviourModuleV1::RecalculatePulseFrequency(unsigned long currentTime, float deltaT);
    SensorModule& _sensors;
    Functions& _f;
    int _sensorIndicator;
};

#endif
