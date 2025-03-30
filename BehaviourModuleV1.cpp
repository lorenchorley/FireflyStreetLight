#include "Arduino.h"
#include "BehaviourModuleV1.h"
#include "Functions.h"
#include "LinkedList.h"

BehaviourModuleV1::BehaviourModuleV1(Functions& f)
{
  _f = f;
}

// Variables
const float twopi = 2 * 3.141592;

// Sensor readings
//int val1 = 0;                               // variable to store the sensor status (value)
//int val2 = 0;                               // variable to store the sensor status (value)
bool previousVal1 = false;
bool previousVal2 = false;
LinkedList<unsigned long> sensorEventTimestamps;
const unsigned long sensorEventTimeout = 20000; // in milliseconds
const int maxSensorEvents = 20;
float recentnessLimit = 0;

float luminosityYellow = 0;                       // current luminosity level
float luminosityRed = 0;                  // current luminosity level of the pilot light
float rateOfChangeRed = 0;                     // current rate of change of luminosity
float accelerationRed = 0;                     // acceleration for the rate of change
float rateOfChangeYellow = 0;                     // current rate of change of luminosity
float accelerationYellow = 0;                     // acceleration for the rate of change
unsigned long motionDetectedTime = 0;       // time when motion was detected
unsigned long stoppedAcceleratingTimeRed = 0;  // time when acceleration was stopped
unsigned long stoppedAcceleratingTimeYellow = 0;  // time when acceleration was stopped
bool finishedAcceleratingRed = false;
bool finishedAcceleratingYellow = false;

// Constants
const float hardMaxLuminosity = 4095.0;       // maximum luminosity value
const float dragCoefficient = 0.05;   // coefficient for the drag effect

// Min/max of colour ranges
const float minLuminosityYellow = 0;
const float minLuminosityRed = 50;
const float maxLuminosityYellow = hardMaxLuminosity;       
const float maxLuminosityRedReal = 0.3 * hardMaxLuminosity; // after the oscillations have been taken into account
const float maxOscillationHeightRed = (maxLuminosityRedReal - minLuminosityRed) / 2;
const float maxLuminosityRed = maxOscillationHeightRed + minLuminosityRed;

// Curve timings
unsigned long holdDuration;        // duration to hold the light at full brightness after motion detected
unsigned long accelerationDurationYellow; // duration to apply acceleration in milliseconds
unsigned long accelerationDurationRed;
float increasingAccelerationValueYellow;  // base acceleration value (adjust as needed)
float increasingAccelerationValueRed;  // base acceleration value (adjust as needed)
float decreasingAccelerationValue;  // base acceleration value (adjust as needed)
float minFrequency;
float maxFrequency;
float frequency;
float targetFrequency;

// State enumeration
enum State {
  IDLE,
  ACCELERATING,
  HOLDING,
  DECELERATING
};

// State variables
State currentState = IDLE;
bool firstInstantOfMotionDetection = false;
bool sensorReportingMotion = false;
float phase = 0;


float CalculateDrag(float value, float rateOfChange, float acceleration, float maxValue) {
  /*float luminosityPercentage = luminosity / maxLuminosity;
  float variableDragCoefficient = dragCoefficient;

  // Apply a resistance to increasing luminosity to give a soft max effect and not arrive too abruptly at the maximum value
  variableDragCoefficient += 0.1 * luminosityPercentage;

  // Apply drag effect regardless of motion detection to avoid excessive rates of change
  float drag = -rateOfChange * variableDragCoefficient;*/

  float drag = -acceleration * 0.1;
  
  return drag;
}

bool BehaviourModuleV1::ReadSensorValues(unsigned long currentTime) {
  bool val1 = _f.ReadFirstSensor();   // read sensor value
  bool val2 = _f.ReadSecondSensor();   // read sensor value

  if (val1 == true && previousVal1 == false) {
    // New sensor event of sensor 1
    previousVal1 = true;
    sensorEventTimestamps.add(currentTime);
  } else if (val1 == false && previousVal1 == true) {
    // Sensor event times out
    previousVal1 = false;
  }
  
  if (val2 == true && previousVal2 == false) {
    // New sensor event of sensor 2
    previousVal2 = true;
    sensorEventTimestamps.add(currentTime);
  } else if (val2 == false && previousVal2 == true) {
    // Sensor event times out
    previousVal2 = false;
  }

  // Delete just one event if there are more than the max
  if (sensorEventTimestamps.size() > maxSensorEvents) {
    sensorEventTimestamps.remove(0);
  }

  // Clean up just the last entry in list if it has exceeded 15s in age
  if (sensorEventTimestamps.size() > 0 && 
      currentTime - sensorEventTimestamps.get(0) > sensorEventTimeout) {
    sensorEventTimestamps.remove(0);
  }

  bool combinedSensorReading = val1 || val2;

  if (firstInstantOfMotionDetection) {
    firstInstantOfMotionDetection = false;
  }

  if (combinedSensorReading == true) {
    if (!sensorReportingMotion) {                   // if motion has not yet been detected, then this is the start of a detection cycle
      sensorReportingMotion = true;                 // set motion detected flag, this allows finer control over the motion detection period rather than leaving it to the sensor
      firstInstantOfMotionDetection = true;
      motionDetectedTime = currentTime;
    }
  }
  else  
  {
    sensorReportingMotion = false;                  // End of a motion detection cycle
  } 

  _f.SetSensorIndicatorPin(sensorReportingMotion);

  return combinedSensorReading;
}

void BehaviourModuleV1::RecalculatePulseFrequency(unsigned long currentTime, float deltaT) {
  
  // Every second recalculate the frequency according the recorded sensor events
  if (_f.IsOncePerSecondEvent()) {
    unsigned long count = 0;
    for (int i = 0; i < sensorEventTimestamps.size(); i++){
      unsigned long timestamp = sensorEventTimestamps.get(i);
      unsigned long age = currentTime - timestamp;
      unsigned long recentness = sensorEventTimeout - age;
      count += recentness;
    }

    // Push back the limit if the count exceeds it to "adapt" to new sensory information
    if (count > recentnessLimit) {
      recentnessLimit = count;
    }
    
    targetFrequency = map(recentnessLimit - count, 0, recentnessLimit, maxFrequency, minFrequency);
    targetFrequency = constrain(targetFrequency, minFrequency, maxFrequency);
  }

  //frequency = lerp(deltaT / 2, frequency, targetFrequency);
  frequency = 0.05 * (targetFrequency - frequency);
  phase += 10 * twopi * frequency * deltaT;

  /*if (phase >= float.max - 10000.0) {
    phase = -float.min;
  }*/
  
}

float CalculatePulseOffset(int height, int min) {  
  //float angle = (twopi * (currentTime)) / frequency;
  float sineValue = sin(phase);

  if (height < min) { height = min; }
  
  int pulseBrightness = (int)((sineValue) * height);
  return pulseBrightness;
}

float lerp(float x, float a, float b)
{ 
  return a + x*(b - a);
}


void BehaviourModuleV1::SetInitialValues() {
//#ifdef DEBUG_TIMINGS
//  accelerationDurationYellow = 6000;
//  accelerationDurationRed = 4000;
//  increasingAccelerationValueYellow = 10;
//  increasingAccelerationValueRed = 15;
//
//  holdDuration = 1000;
//  
//  decreasingAccelerationValue = -10; 
//
//  minFrequency = 0.5;
//  maxFrequency = 2;
//#else
  accelerationDurationYellow = 5000;
  accelerationDurationRed = 7000;
  increasingAccelerationValueYellow = 7;
  increasingAccelerationValueRed = 3;

  holdDuration = 8000;
  
  decreasingAccelerationValue = -12;

  minFrequency = 0.3;
  maxFrequency = 1.5;
//#endif

  frequency = maxFrequency;
}

void BehaviourModuleV1::Tick(unsigned long currentTime, float deltaT) 
{
  _f.Update(currentTime);
  bool sensorReading = ReadSensorValues(currentTime);
  RecalculatePulseFrequency(currentTime, deltaT);

  switch (currentState) {
    case IDLE:
      accelerationRed = 0;
      accelerationYellow = 0;
      rateOfChangeRed = 0;
      rateOfChangeYellow = 0;
    
      if (firstInstantOfMotionDetection) {
        currentState = ACCELERATING;
      }
      
      break;

    case ACCELERATING:
    
      accelerationRed = increasingAccelerationValueRed;
      accelerationYellow = increasingAccelerationValueYellow;

      if (sensorReading == true) {
        break;
      }
      
      if (currentTime - motionDetectedTime < accelerationDurationRed) {
        rateOfChangeRed += accelerationRed;
      } else if (!finishedAcceleratingRed) {
        stoppedAcceleratingTimeRed = currentTime;
        accelerationRed = 0;
        rateOfChangeRed = 0;
        finishedAcceleratingRed = true;
      }
      
      if (currentTime - motionDetectedTime < accelerationDurationYellow) {
        rateOfChangeYellow += accelerationYellow;
      } else if (!finishedAcceleratingYellow) {
        stoppedAcceleratingTimeYellow = currentTime;
        accelerationYellow = 0;
        rateOfChangeYellow = 0;        
        finishedAcceleratingYellow = true;
      }

      if (finishedAcceleratingRed && finishedAcceleratingYellow) 
      {
        currentState = HOLDING;
        finishedAcceleratingRed = false;
        finishedAcceleratingYellow = false;
        break;
      }
      
      break;

    case HOLDING:
      accelerationRed = 0;
      accelerationYellow = 0;
      rateOfChangeRed = 0;
      rateOfChangeYellow = 0;
      
      if (firstInstantOfMotionDetection) {
        currentState = ACCELERATING;
      }
      
      if (currentTime - stoppedAcceleratingTimeRed > holdDuration || 
          currentTime - stoppedAcceleratingTimeYellow > holdDuration) {
        currentState = DECELERATING;
      }
      
      break;

    case DECELERATING:
      accelerationRed = decreasingAccelerationValue;
      accelerationYellow = decreasingAccelerationValue;

      if (firstInstantOfMotionDetection) {
        currentState = ACCELERATING;
        break;
      } 

      if (luminosityRed <= minLuminosityRed && luminosityYellow <= minLuminosityYellow) {
        currentState = IDLE;
        break;
      }
      
      break;
  }

  // Calculate rate of change (currently missing deltaT calculation so wont work the same with different clock cycle or loop delay)
  rateOfChangeRed += (accelerationRed + CalculateDrag(luminosityRed, rateOfChangeRed, accelerationRed, maxLuminosityRed)) * deltaT;
  rateOfChangeYellow += (accelerationYellow + CalculateDrag(luminosityYellow, rateOfChangeYellow, accelerationYellow, maxLuminosityYellow)) * deltaT;

  // Apply rate of change to the luminosity
  luminosityRed += rateOfChangeRed * deltaT;
  luminosityYellow += rateOfChangeYellow * deltaT;

  // Apply min/max luminosity constraints
  if (luminosityRed > maxLuminosityRed || luminosityRed < minLuminosityRed) {
    rateOfChangeRed = 0;
  }  
  luminosityRed = constrain(luminosityRed, minLuminosityRed, maxLuminosityRed);

  if (luminosityYellow > maxLuminosityYellow || luminosityYellow < minLuminosityYellow) {
    rateOfChangeYellow = 0;
  }
  luminosityYellow = constrain(luminosityYellow, minLuminosityYellow, maxLuminosityYellow);

  // Apply fluctuations and other post-physical and/or non persisting modification calculations
  float pulseOffset = CalculatePulseOffset((int)lerp((luminosityRed - minLuminosityRed) / (maxLuminosityRed - minLuminosityRed), 20, maxOscillationHeightRed), 10);

  // Final transformation into arduino units (0-255) before writing as PWN signal
  //int finalRed = (int)map(luminosityRed + pulseOffset, 0, hardMaxLuminosity, 0, 255);
  //int finalYellow = (int)map(luminosityYellow, 0, hardMaxLuminosity, 0, 255);
  uint16_t finalRed = (uint16_t)map(luminosityRed + pulseOffset, 0, hardMaxLuminosity, 0, 4095);
  uint16_t finalYellow = (uint16_t)map(luminosityYellow, 0, hardMaxLuminosity, 0, 4095);
  
  //analogWrite(ledYellow, finalYellow);
  //analogWrite(ledRed, finalRed);
  _f.SetPWMOnPin9(finalRed);
  _f.SetPWMOnPin10(finalYellow);

  if (true) {
#ifndef ATTINY
    if (false) {
      Serial.print("Rate_of_Change_Yellow:");
      Serial.print(rateOfChangeYellow);
      Serial.print(",");
      Serial.print("Rate_of_Change_Red:");
      Serial.print(rateOfChangeRed);
      Serial.print(",");
      Serial.print("accelerationYellow:");
      Serial.print(accelerationYellow);
      Serial.print(",");
      Serial.print("accelerationRed:");
      Serial.print(accelerationRed);
      Serial.print(",");
    }
    
    if (true) {
      Serial.print("Sensor:");
      Serial.print(sensorReportingMotion ? "10" : "0");
      Serial.print(",");
    }
    
    if (false) {
      Serial.print("State:");
      Serial.print(-((int)currentState) * 10);
      Serial.print(",");
    }
  
    if (false) {
      Serial.print("deltaT:");
      Serial.print(deltaT * 100);
      Serial.print(",");
    }
    
    if (false) {
      Serial.print("pulseOffset:");
      Serial.print(pulseOffset / 4);
      Serial.print(",");
    }
    
    if (true) {
      Serial.print("targetFrequency:");
      Serial.print(targetFrequency * 10);
      Serial.print(",");
      Serial.print("Frequency:");
      Serial.print(frequency * 10);
      Serial.print(",");
    }
    
    Serial.print("Yellow:");
    Serial.print(finalYellow);
    Serial.print(",");
    Serial.print("Red:");
    Serial.print(finalRed);
    Serial.println(",");
#endif
  }
}
