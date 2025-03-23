#ifndef Functions_h
#define Functions_h
#include "Arduino.h"

class Functions
{
  public:
    Functions();
    void Functions::Update(unsigned long currentTime);
    bool Functions::IsOncePerSecondEvent();
    void Functions::SetPWMOnPin9(uint16_t value);
    void Functions::SetPWMOnPin10(uint16_t value);
};

#endif
