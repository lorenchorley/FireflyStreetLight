
using FireflyEnvDev.Simulators;

namespace FireflyEnvDev.Inputs;

internal class MovementSensorInput
{
    bool isDone = false;

    internal SensorReading Read(TimeSpan timeElapsed)
    {
        return (!isDone && timeElapsed.TotalSeconds > 1) 
            ? SensorReading.HIGH 
            : SensorReading.LOW;
    }

    internal void SetEvents(params TimeSpan[] timeSpans)
    {
        throw new NotImplementedException();
    }
}
