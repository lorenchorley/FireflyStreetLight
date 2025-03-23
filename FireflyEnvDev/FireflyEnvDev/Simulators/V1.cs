
using FireflyEnvDev.Inputs;
using FireflyEnvDev.Memory;
using FireflyEnvDev.Outputs;
using FireflyEnvDev.Simulators.General;
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
    public Functions Functions { get; private set; }    

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
        Functions = new Functions(this);

        TickDuration = tickDuration;
    }

    public void Start()
    {
        
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
