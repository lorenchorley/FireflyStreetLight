#include "Functions.h"
#include "BehaviourModuleV1.h"

//#define UNO
#define NANO
//#define ATTINY

//#define DEBUG_TIMINGS

// Pins

#ifdef UNO
int led = 10;
int sensor = 12;
int sensorIndicatorPin = 1;
#endif

#ifdef NANO
int ledYellow = 10; // Digital output PWM
int ledRed = 9; // Digital output PWM, with register specific code
int sensor1 = 7; // Digital input
int sensor2 = 8; // Digital input
int sensorIndicatorPin = LED_BUILTIN;
#endif

#ifdef ATTINY
int ledYellow = 0; // Digital output PWM
int ledRed = 1; // Digital output PWM
int sensor1 = 2; // Digital input
int sensor2 = 3; // Digital input
int sensorIndicatorPin = 4; // Digital/analog output 
#endif

void FastPWMSetup() {
  // Configure Timer 1 for Fast PWM mode
  TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1); // Fast PWM, clear OC1A and OC1B on compare match
  TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10); // Fast PWM, no prescaling

  // Set the ICR1 value for a top value (for 12-bit resolution)
  ICR1 = 4095; // Set TOP value for 12-bit resolution
}

Functions f(sensor1, sensor2, sensorIndicatorPin);
BehaviourModuleV1 Simulation(f);

float deltaT;
unsigned long currentTime;
unsigned long startupPeriodFinished = 0;    // The time at the state machine can start
const unsigned long startupPeriod = 2;            // Time for the components to initialise before the state machine starts, in seconds

void ReadTime() {
  unsigned long now = millis();
  deltaT = (now - currentTime)/1000.0;
  currentTime = now;
}

void setup() {
  Simulation.SetInitialValues();

  pinMode(ledYellow, OUTPUT);      
  pinMode(ledRed, OUTPUT);      
  pinMode(sensorIndicatorPin, OUTPUT);   
  pinMode(sensor1, INPUT);    
  pinMode(sensor2, INPUT);  

  FastPWMSetup();
    
#ifndef ATTINY
  Serial.begin(9600);
#endif        

  startupPeriodFinished = millis() + startupPeriod * 1000;
}

void loop() {
  ReadTime();
  
  // No behaviour durion startup phase
  if (currentTime < startupPeriodFinished) {
    return; 
  }

  Simulation.Tick(currentTime, deltaT);

  delay(2);  // small delay to make the loop more stable
}
