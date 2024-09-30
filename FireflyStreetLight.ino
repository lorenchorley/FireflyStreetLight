#include "LinkedList.h"

//#define UNO
#define NANO
//#define ATTINY

//#define DEBUG_TIMINGS

// Pins

#ifdef UNO
int led = 10;
int sensor = 12;
int sensorIndicator = 1;
#endif

#ifdef NANO
int ledYellow = 10; // Digital output PWM
int ledRed = 9; // Digital output PWM, with register specific code
int sensor1 = 7; // Digital input
int sensor2 = 8; // Digital input
int sensorIndicator = LED_BUILTIN;
#endif

#ifdef ATTINY
int ledYellow = 0; // Digital output PWM
int ledRed = 1; // Digital output PWM
int sensor1 = 2; // Digital input
int sensor2 = 3; // Digital input
int sensorIndicator = 4; // Digital/analog output 
#endif


// Variables
unsigned long startupPeriodFinished = 0;    // The time at the state machine can start
const float twopi = 2 * 3.141592;

// Sensor readings
int val1 = 0;                               // variable to store the sensor status (value)
int val2 = 0;                               // variable to store the sensor status (value)
int previousVal1 = 0;
int previousVal2 = 0;
LinkedList<unsigned long> sensorEventTimestamps;
const unsigned long sensorEventTimeout = 20000; // in milliseconds
const int maxSensorEvents = 20;
float recentnessLimit = 0;

int combinedSensorReading = 0;              //
float luminosityYellow = 0;                       // current luminosity level
float luminosityRed = 0;                  // current luminosity level of the pilot light
float rateOfChangeRed = 0;                     // current rate of change of luminosity
float accelerationRed = 0;                     // acceleration for the rate of change
float rateOfChangeYellow = 0;                     // current rate of change of luminosity
float accelerationYellow = 0;                     // acceleration for the rate of change
unsigned long motionDetectedTime = 0;       // time when motion was detected
unsigned long stoppedAcceleratingTimeRed = 0;  // time when acceleration was stopped
unsigned long stoppedAcceleratingTimeYellow = 0;  // time when acceleration was stopped
unsigned long currentTime;              // current time
unsigned long previousTime;
float deltaT;
float i = 0;
bool finishedAcceleratingRed = false;
bool finishedAcceleratingYellow = false;
unsigned long previousWholeSecondsFigure = 0;
bool OncePerSecondEvent = false;

// Constants
const unsigned long startupPeriod = 2;            // Time for the components to initialise before the state machine starts, in seconds
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
int holdDuration;        // duration to hold the light at full brightness after motion detected
int accelerationDurationYellow; // duration to apply acceleration in milliseconds
int accelerationDurationRed;
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

void FastPWMSetup() {
  // Configure Timer 1 for Fast PWM mode
  TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1); // Fast PWM, clear OC1A and OC1B on compare match
  TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10); // Fast PWM, no prescaling

  // Set the ICR1 value for a top value (for 12-bit resolution)
  ICR1 = 4095; // Set TOP value for 12-bit resolution
}

void setPWMOnPin9(uint16_t value) {
  // Ensure value is within 0-4095
  if (value > 4095) {
    value = 4095;
  } else if (value < 0) {
    value = 0;
  }
  
  // Set the Output Compare Register for Timer 1
  OCR1A = value; // Set the PWM value for pin 9
}

void setPWMOnPin10(uint16_t value) {
  // Ensure value is within 0-4095
  if (value > 4095) {
    value = 4095;
  } else if (value < 0) {
    value = 0;
  }
  
  // Set the Output Compare Register for Timer 1
  OCR1B = value; // Set the PWM value for pin 10
}

void setup() {
#ifdef DEBUG_TIMINGS
  accelerationDurationYellow = 6000;
  accelerationDurationRed = 4000;
  increasingAccelerationValueYellow = 10;
  increasingAccelerationValueRed = 15;

  holdDuration = 1000;
  
  decreasingAccelerationValue = -10; 

  minFrequency = 0.5;
  maxFrequency = 2;
#else
  accelerationDurationYellow = 5000;
  accelerationDurationRed = 7000;
  increasingAccelerationValueYellow = 7;
  increasingAccelerationValueRed = 3;

  holdDuration = 8000;
  
  decreasingAccelerationValue = -12;

  minFrequency = 0.3;
  maxFrequency = 1.5;
#endif

  frequency = maxFrequency;
  
  pinMode(ledYellow, OUTPUT);      
  pinMode(ledRed, OUTPUT);      
  pinMode(sensorIndicator, OUTPUT);   
  pinMode(sensor1, INPUT);    
  pinMode(sensor2, INPUT);  

  FastPWMSetup();

  startupPeriodFinished = millis() + startupPeriod * 1000;
    
#ifndef ATTINY
  Serial.begin(9600);
#endif        

}

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

void ReadSensorValues() {
  val1 = digitalRead(sensor1);   // read sensor value
  val2 = digitalRead(sensor2);   // read sensor value

  if (val1 == HIGH && previousVal1 == LOW) {
    // New sensor event of sensor 1
    previousVal1 = HIGH;
    sensorEventTimestamps.add(currentTime);
  } else if (val1 == LOW && previousVal1 == HIGH) {
    // Sensor event times out
    previousVal1 = LOW;
  }
  
  if (val2 == HIGH && previousVal2 == LOW) {
    // New sensor event of sensor 2
    previousVal2 = HIGH;
    sensorEventTimestamps.add(currentTime);
  } else if (val2 == LOW && previousVal2 == HIGH) {
    // Sensor event times out
    previousVal2 = LOW;
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

  combinedSensorReading = val1 || val2;

  if (firstInstantOfMotionDetection) {
    firstInstantOfMotionDetection = false;
  }

  if (combinedSensorReading == HIGH) {
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

  digitalWrite(sensorIndicator, sensorReportingMotion ? HIGH : LOW);
}

float phase = 0;

void RecalculatePulseFrequency() {
  
  // Every second recalculate the frequency according the recorded sensor events
  if (OncePerSecondEvent) {
    long count = 0;
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

void ReadTime() {
  unsigned long now = millis();
  deltaT = (now - currentTime)/1000.0;
  previousTime = currentTime;
  currentTime = now;

  unsigned long wholeSecondsFigure = now / 1000;
  OncePerSecondEvent = previousWholeSecondsFigure != wholeSecondsFigure;
  previousWholeSecondsFigure = wholeSecondsFigure; 
}

float lerp(float x, float a, float b)
{ 
  return a + x*(b - a);
}

void loop() {
  ReadTime();
  
  // No behaviour durion startup phase
  if (currentTime < startupPeriodFinished) {
    return; 
  }

  ReadSensorValues();
  RecalculatePulseFrequency();

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

      if (combinedSensorReading == HIGH) {
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
  float pulseOffset = CalculatePulseOffset(lerp((luminosityRed - minLuminosityRed) / (maxLuminosityRed - minLuminosityRed), 20, maxOscillationHeightRed), 10);

  // Final transformation into arduino units (0-255) before writing as PWN signal
  //int finalRed = (int)map(luminosityRed + pulseOffset, 0, hardMaxLuminosity, 0, 255);
  //int finalYellow = (int)map(luminosityYellow, 0, hardMaxLuminosity, 0, 255);
  uint16_t finalRed = (uint16_t)map(luminosityRed + pulseOffset, 0, hardMaxLuminosity, 0, 4095);
  uint16_t finalYellow = (uint16_t)map(luminosityYellow, 0, hardMaxLuminosity, 0, 4095);
  
  //analogWrite(ledYellow, finalYellow);
  //analogWrite(ledRed, finalRed);
  setPWMOnPin9(finalRed);
  setPWMOnPin10(finalYellow);

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
      Serial.print(-currentState * 10);
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

  delay(2);  // small delay to make the loop more stable
}
