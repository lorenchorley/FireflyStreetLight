
using FireflyEnvDev.Inputs;
using FireflyEnvDev.Memory;
using FireflyEnvDev.Outputs;
using FireflyEnvDev.Simulators.Interfaces;

namespace FireflyEnvDev.Simulators;

public enum State
{
    IDLE,
    ACCELERATING,
    HOLDING,
    DECELERATING
};

public enum SensorReading
{
    LOW,
    HIGH
}

internal class V1 : ISimulator
{
    public TimeSpan TimeElapsed { get; set; }
    public TimeSpan TickDuration { get; private set; }

    public MovementSensorInput MovementSensorInput1 { get; private set; }
    public MovementSensorInput MovementSensorInput2 { get; private set; }
    public GraphDataOutput GraphDataOutput { get; private set; }
    public LEDOutput YellowLEDOutput { get; private set; }
    public LEDOutput RedLEDOutput { get; private set; }
    public SerialOutput SerialOutput { get; private set; }

    public RollOverEventList RollOverEventList { get; private set; }

    // Variables
    ulong startupPeriodFinished = 0;    // The time at the state machine can start
    const float twopi = 2 * 3.141592f;

    // Sensor readings
    int val1 = 0;                               // variable to store the sensor status (value)
    int val2 = 0;                               // variable to store the sensor status (value)
    int previousVal1 = 0;
    int previousVal2 = 0;
    LinkedList<ulong> sensorEventTimestamps;
    const ulong sensorEventTimeout = 20000; // in milliseconds
    const int maxSensorEvents = 20;
    float recentnessLimit = 0;

    int combinedSensorReading = 0;              //
    float luminosityYellow = 0;                       // current luminosity level
    float luminosityRed = 0;                  // current luminosity level of the pilot light
    float rateOfChangeRed = 0;                     // current rate of change of luminosity
    float accelerationRed = 0;                     // acceleration for the rate of change
    float rateOfChangeYellow = 0;                     // current rate of change of luminosity
    float accelerationYellow = 0;                     // acceleration for the rate of change
    ulong motionDetectedTime = 0;       // time when motion was detected
    ulong stoppedAcceleratingTimeRed = 0;  // time when acceleration was stopped
    ulong stoppedAcceleratingTimeYellow = 0;  // time when acceleration was stopped
    ulong currentTime;              // current time
                                            
    float deltaT;
    float i = 0;
    bool finishedAcceleratingRed = false;
    bool finishedAcceleratingYellow = false;
    ulong previousWholeSecondsFigure = 0;
    bool OncePerSecondEvent = false;

    // Constants
    const ulong startupPeriod = 2;            // Time for the components to initialise before the state machine starts, in seconds
    const float hardMaxLuminosity = 4095.0f;       // maximum luminosity value
    const float dragCoefficient = 0.05f;   // coefficient for the drag effect

    // Min/max of colour ranges
    const float minLuminosityYellow = 0;
    const float minLuminosityRed = 50;
    const float maxLuminosityYellow = hardMaxLuminosity;
    const float maxLuminosityRedReal = 0.3f * hardMaxLuminosity; // after the oscillations have been taken into account
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

    // State variables
    State currentState = State.IDLE;
    bool firstInstantOfMotionDetection = false;
    bool sensorReportingMotion = false;

    float CalculateDrag(float value, float rateOfChange, float acceleration, float maxValue)
    {
        /*float luminosityPercentage = luminosity / maxLuminosity;
        float variableDragCoefficient = dragCoefficient;

        // Apply a resistance to increasing luminosity to give a soft max effect and not arrive too abruptly at the maximum value
        variableDragCoefficient += 0.1 * luminosityPercentage;

        // Apply drag effect regardless of motion detection to avoid excessive rates of change
        float drag = -rateOfChange * variableDragCoefficient;*/

        float drag = -acceleration * 0.1f;

        return drag;
    }

    //void ReadSensorValues()
    //{
    //    val1 = MovementSensorInput1.Read(TimeElapsed);   // read sensor value
    //    val2 = digitalRead(sensor2);   // read sensor value

    //    if (val1 == HIGH && previousVal1 == LOW)
    //    {
    //        // New sensor event of sensor 1
    //        previousVal1 = HIGH;
    //        sensorEventTimestamps.add(currentTime);
    //    }
    //    else if (val1 == LOW && previousVal1 == HIGH)
    //    {
    //        // Sensor event times out
    //        previousVal1 = LOW;
    //    }

    //    if (val2 == HIGH && previousVal2 == LOW)
    //    {
    //        // New sensor event of sensor 2
    //        previousVal2 = HIGH;
    //        sensorEventTimestamps.add(currentTime);
    //    }
    //    else if (val2 == LOW && previousVal2 == HIGH)
    //    {
    //        // Sensor event times out
    //        previousVal2 = LOW;
    //    }

    //    // Delete just one event if there are more than the max
    //    if (sensorEventTimestamps.size() > maxSensorEvents)
    //    {
    //        sensorEventTimestamps.remove(0);
    //    }

    //    // Clean up just the last entry in list if it has exceeded 15s in age
    //    if (sensorEventTimestamps.size() > 0 &&
    //        currentTime - sensorEventTimestamps.get(0) > sensorEventTimeout)
    //    {
    //        sensorEventTimestamps.remove(0);
    //    }

    //    combinedSensorReading = val1 || val2;

    //    if (firstInstantOfMotionDetection)
    //    {
    //        firstInstantOfMotionDetection = false;
    //    }

    //    if (combinedSensorReading == HIGH)
    //    {
    //        if (!sensorReportingMotion)
    //        {                   // if motion has not yet been detected, then this is the start of a detection cycle
    //            sensorReportingMotion = true;                 // set motion detected flag, this allows finer control over the motion detection period rather than leaving it to the sensor
    //            firstInstantOfMotionDetection = true;
    //            motionDetectedTime = currentTime;
    //        }
    //    }
    //    else
    //    {
    //        sensorReportingMotion = false;                  // End of a motion detection cycle
    //    }

    //    digitalWrite(sensorIndicator, sensorReportingMotion ? HIGH : LOW);
    //}

    //float phase = 0;

    //void RecalculatePulseFrequency()
    //{

    //    // Every second recalculate the frequency according the recorded sensor events
    //    if (OncePerSecondEvent)
    //    {
    //        long count = 0;
    //        for (int i = 0; i < sensorEventTimestamps.size(); i++)
    //        {
    //            unsigned long timestamp = sensorEventTimestamps.get(i);
    //            unsigned long age = currentTime - timestamp;
    //            unsigned long recentness = sensorEventTimeout - age;
    //            count += recentness;
    //        }

    //        // Push back the limit if the count exceeds it to "adapt" to new sensory information
    //        if (count > recentnessLimit)
    //        {
    //            recentnessLimit = count;
    //        }

    //        targetFrequency = map(recentnessLimit - count, 0, recentnessLimit, maxFrequency, minFrequency);
    //        targetFrequency = constrain(targetFrequency, minFrequency, maxFrequency);
    //    }

    //    //frequency = lerp(deltaT / 2, frequency, targetFrequency);
    //    frequency = 0.05 * (targetFrequency - frequency);
    //    phase += 10 * twopi * frequency * deltaT;

    //    /*if (phase >= float.max - 10000.0) {
    //      phase = -float.min;
    //    }*/

    //}

    //float CalculatePulseOffset(int height, int min)
    //{
    //    //float angle = (twopi * (currentTime)) / frequency;
    //    float sineValue = sin(phase);

    //    if (height < min) { height = min; }

    //    int pulseBrightness = (int)((sineValue) * height);
    //    return pulseBrightness;
    //}

    //void ReadTime()
    //{
    //    unsigned long now = millis();
    //    deltaT = (now - currentTime) / 1000.0;
    //    //previousTime = currentTime;
    //    currentTime = now;

    //    unsigned long wholeSecondsFigure = now / 1000;
    //    OncePerSecondEvent = previousWholeSecondsFigure != wholeSecondsFigure;
    //    previousWholeSecondsFigure = wholeSecondsFigure;
    //}

    private float lerp(float x, float a, float b)
    {
        return a + x * (b - a);
    }

    public V1(TimeSpan tickDuration)
    {
        TimeElapsed = TimeSpan.FromSeconds(0);
        
        MovementSensorInput1 = new MovementSensorInput();
        MovementSensorInput2 = new MovementSensorInput();
        GraphDataOutput = new GraphDataOutput();
        YellowLEDOutput = new LEDOutput();
        RedLEDOutput = new LEDOutput();
        SerialOutput = new SerialOutput();

        RollOverEventList = new RollOverEventList();
        TickDuration = tickDuration;
    }

    public void Start()
    {
        accelerationDurationYellow = 5000;
        accelerationDurationRed = 7000;
        increasingAccelerationValueYellow = 7;
        increasingAccelerationValueRed = 3;

        holdDuration = 8000;

        decreasingAccelerationValue = -12;

        minFrequency = 0.3f;
        maxFrequency = 1.5f;

        // Initialise
        frequency = maxFrequency;

        GraphDataOutput.RegisterTimeSeriesDataNames(
            "S1",
            "S2"
        );
    }

    public void Loop()
    {
        bool seesMovement = MovementSensorInput1.Read(TimeElapsed) == SensorReading.HIGH;
        if (seesMovement)
        {
            SerialOutput.WriteLine($"Saw movement at {TimeElapsed}");
        }

        GraphDataOutput.WriteTimeSeriesDataPoint(TimeElapsed, 
            ( "S1", MathF.Sin((float)TimeElapsed.TotalMilliseconds / 1000) ),
            ( "S2", MathF.Cos((float)TimeElapsed.TotalMilliseconds / 1000))
        );
    }
}
