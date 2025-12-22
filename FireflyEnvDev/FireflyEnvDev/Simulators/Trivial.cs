
using FireflyEnvDev.Inputs;
using FireflyEnvDev.Memory;
using FireflyEnvDev.Outputs;
using FireflyEnvDev.Simulators.Interfaces;

namespace FireflyEnvDev.Simulators;

internal class Trivial : ISimulator
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

    public Trivial(TimeSpan tickDuration)
    {
        TimeElapsed = TimeSpan.FromSeconds(0);
        
        MovementSensorInput1 = new MovementSensorInput();
        MovementSensorInput2 = new MovementSensorInput();
        YellowLEDOutput = new LEDOutput();
        RedLEDOutput = new LEDOutput();
        SerialOutput = new Serial();
        GraphDataOutput = new GraphDataOutput(SerialOutput);

        RollOverEventList = new RollOverEventList();
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
    }

    public void Loop()
    {
        bool seesMovement = 
            MovementSensorInput1.Read(TimeElapsed) ||
            MovementSensorInput2.Read(TimeElapsed);

        if (seesMovement)
        {
            SerialOutput.println($"Saw movement at {TimeElapsed}");
        }

        GraphDataOutput.WriteTimeSeriesDataPoint(TimeElapsed, 
            ( "S1", MathF.Sin((float)TimeElapsed.TotalMilliseconds / 1000) ),
            ( "S2", MathF.Cos((float)TimeElapsed.TotalMilliseconds / 1000))
        );
    }
}
