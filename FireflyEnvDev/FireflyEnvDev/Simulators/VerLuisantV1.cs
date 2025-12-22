
using FireflyEnvDev.Inputs;
using FireflyEnvDev.Memory;
using FireflyEnvDev.Outputs;
using FireflyEnvDev.Simulators.FromArduino;
using FireflyEnvDev.Simulators.General;
using FireflyEnvDev.Simulators.Interfaces;

namespace FireflyEnvDev.Simulators;

//public enum State
//{
//    IDLE,
//    ACCELERATING,
//    HOLDING,
//    DECELERATING
//};

//public enum SensorReading
//{
//    LOW,
//    HIGH
//}

internal class VerLuisantV1 : ISimulator
{
    public TimeSpan TimeElapsed { get; set; }
    public TimeSpan TickDuration { get; private set; }

    public MovementSensorInput MovementSensorInput1 { get; private set; }
    public MovementSensorInput MovementSensorInput2 { get; private set; }
    public GraphDataOutput GraphDataOutput { get; private set; }
    public LEDOutput YellowLEDOutput { get; private set; }
    public LEDOutput RedLEDOutput { get; private set; }
    public Serial SerialOutput { get; private set; }

    public RollOverEventList RollOverEventList { get; private set; }
    public Functions Functions { get; private set; } 
    
    public BehaviourModuleV1 BehaviourModule { get; private set; }    

    public VerLuisantV1(TimeSpan tickDuration)
    {
        TimeElapsed = TimeSpan.Zero;
        
        MovementSensorInput1 = new();
        MovementSensorInput2 = new();
        YellowLEDOutput = new();
        RedLEDOutput = new();
        SerialOutput = new();
        GraphDataOutput = new(SerialOutput);

        RollOverEventList = new();
        Functions = new(this);

        BehaviourModule = new(Functions);

        TickDuration = tickDuration;
    }

    public void Start()
    {
        GraphDataOutput.RegisterTimeSeriesDataNames(
            "Sensor",
            "targetFrequency",
            "Frequency",
            "Yellow",
            "Red"
        );

        BehaviourModule.SetInitialValues();
    }

    public void Loop()
    {
        float deltaT = (float)TickDuration.TotalMilliseconds; // TODO use a stopwatch and do a real calculation
        BehaviourModule.Tick(currentTime: (ulong)TimeElapsed.TotalMilliseconds, deltaT: deltaT);
    }
}
