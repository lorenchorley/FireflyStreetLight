using FireflyEnvDev.Inputs;
using FireflyEnvDev.Outputs;

namespace FireflyEnvDev.Simulators.Interfaces;

internal interface ISimulator
{
    MovementSensorInput MovementSensorInput1 { get; }
    MovementSensorInput MovementSensorInput2 { get; }
    GraphDataOutput GraphDataOutput { get; }
    LEDOutput YellowLEDOutput { get; }
    LEDOutput RedLEDOutput { get; }
    Serial SerialOutput { get; }
    TimeSpan TimeElapsed { get; set; }
    TimeSpan TickDuration { get; }
    void Start();
    void Loop();
}
