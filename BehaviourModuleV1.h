#ifndef BehaviourModuleV1_h
#define BehaviourModuleV1_h
#include "Arduino.h"
#include "Functions.h"

class BehaviourModuleV1
{
  public:
    BehaviourModuleV1(Functions& f);
    void SetInitialValues();
    void Tick(unsigned long currentTime, float deltaT);
  private:
    bool BehaviourModuleV1::ReadSensorValues(unsigned long currentTime);
    void BehaviourModuleV1::RecalculatePulseFrequency(unsigned long currentTime, float deltaT);
    Functions& _f;
};

#endif
