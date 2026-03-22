// ===== PWM pins =====
const int PIN_R = 10;
const int PIN_G = 9;
const int PIN_B = 6;
const int PIN_W = 5;

struct ColorPoint {
  float time;   // arbitrary increasing numbers (keyframe scale)
  int r;
  int g;
  int b;
  int w;
};

// ===== USER SETTINGS =====
const float TOTAL_DAY_SECONDS = 120.0;     // total simulated day length
const float FADE_BACK_DURATION = 0.0;    // in keyframe units, scales automatically
const int INITIAL_KEYFRAME = 190;

// Arbitrary keyframe scale
ColorPoint points[] = {
  {0,  50,0,0,0},        // pre sunrie
  {70,    150,80,0,0},    // sunrise
  {100,  200,140,0,60},  //  morning
  {150,  200,200,255,255}, // noon
  {190,  250,180,10,80},  // afternoon
  {230,  250,100,10,10},   // sunset
  {245,  120,0,05,0},        // 
  {260,  55,0,05,0},        // blue night
  {290,  00,0,0,0},        // 
  {310,  00,0,0,0},        //
  {330,  30,0,0,0},        // pre sunrie
   {350,  50,0,0,0},        // 
};

//ColorPoint points[] = {
////  {0,    0,0,0,0},    // testing at max
////  {50,    100,100,100,100},    // testing at max
////  {100,    0,0,0,0},    // testing at max
//  {0,    100,100,100,100},    // testing at max
//  {50,    255,255,255,255},    // testing at max
//  {100,    100,100,100,100},    // testing at max
//};

const int numPoints = sizeof(points)/sizeof(points[0]);

float minTime, maxTime, scaleFactor;

void setColor(int r,int g,int b,int w) {
  analogWrite(PIN_R,r);
  analogWrite(PIN_G,g);
  analogWrite(PIN_B,b);
  analogWrite(PIN_W,w);
}

void setup() {
  pinMode(PIN_R,OUTPUT);
  pinMode(PIN_G,OUTPUT);
  pinMode(PIN_B,OUTPUT);
  pinMode(PIN_W,OUTPUT);

  Serial.begin(115200);

  // print column labels once
  Serial.println("red green blue white segment phase");

  // find min and max in keyframes
  minTime = points[0].time;
  maxTime = points[numPoints-1].time + FADE_BACK_DURATION; // include fade-back
  scaleFactor = TOTAL_DAY_SECONDS / (maxTime - minTime);
}

void loop() {
  unsigned long t = millis() % (unsigned long)(TOTAL_DAY_SECONDS * 1000UL);
  float currentSec = (float)t / 1000.0;

  int r=0,g=0,b=0,w=0;
  int segment = 0;

  // scale currentSec to the arbitrary keyframe scale
  float scaledTime = currentSec / scaleFactor + minTime;

  // Determine current segment
  bool found = false;
  for(int i=0;i<numPoints-1;i++) {
    if(scaledTime >= points[i].time && scaledTime <= points[i+1].time) {
      segment = i;
      float span = points[i+1].time - points[i].time;
      float localT = (scaledTime - points[i].time)/span;

      float rf = points[i].r + (points[i+1].r - points[i].r) * localT;
      float gf = points[i].g + (points[i+1].g - points[i].g) * localT;
      float bf = points[i].b + (points[i+1].b - points[i].b) * localT;
      float wf = points[i].w + (points[i+1].w - points[i].w) * localT;

      r = round(rf);
      g = round(gf);
      b = round(bf);
      w = round(wf);

      found = true;
      break;
    }
  }

  // If beyond last keyframe, interpolate back to first
  if(!found) {
    segment = numPoints-1;
    float span = FADE_BACK_DURATION;
    float localT = (scaledTime - points[numPoints-1].time) / span;

    float rf = points[numPoints-1].r + (points[0].r - points[numPoints-1].r) * localT;
    float gf = points[numPoints-1].g + (points[0].g - points[numPoints-1].g) * localT;
    float bf = points[numPoints-1].b + (points[0].b - points[numPoints-1].b) * localT;
    float wf = points[numPoints-1].w + (points[0].w - points[numPoints-1].w) * localT;

    r = round(rf);
    g = round(gf);
    b = round(bf);
    w = round(wf);
  }

  setColor(r,g,b,w);

  // ===== DAY PHASE SINE =====
  // phase 0 -> 2π over full day
  float phase = (currentSec / TOTAL_DAY_SECONDS) * 2.0 * 3.14159265;
  float sinVal = sin(phase);
  // map from [-1,1] to [0,255] for plotting
  int sinPlot = round((sinVal + 1.0) * 127.5);

  // ===== SERIAL PLOTTER OUTPUT WITH LABELS =====
  Serial.print(r); Serial.print(" ");
  Serial.print(g); Serial.print(" ");
  Serial.print(b); Serial.print(" ");
  Serial.print(w); Serial.print(" ");
  Serial.print(segment); Serial.print(" ");
  Serial.println(sinPlot);

  delay(20);
}
