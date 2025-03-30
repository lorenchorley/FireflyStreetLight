using FireflyEnvDev.Outputs;
using FireflyEnvDev.Simulators.Interfaces;

namespace FireflyEnvDev.Simulators.General;

public delegate float MapDelegate(float x, float in_min, float in_max, float out_min, float out_max);
public delegate float ConstrainDelegate(float x, float in_min, float in_max);
public delegate float SinDelegate(float x);
public delegate float CosDelegate(float x);

internal class Functions
{
    private readonly ISimulator _simulator;

    public Functions(ISimulator simulator)
    {
        _simulator = simulator;
    }

    #region Arduino library functions
    public Serial Serial => _simulator.SerialOutput;

    public float map(float x, float in_min, float in_max, float out_min, float out_max)
    {
        return 0;
    }
    
    public float constrain(float x, float in_min, float in_max)
    {
        return 0;
    }

    public float sin(float x)
    {
        return 0;
    }

    public float cos(float x)
    {
        return 0;
    }
    #endregion

    #region
    public bool ReadFirstSensor()
    {
        return _simulator.MovementSensorInput1.Read(_simulator.TimeElapsed) == SensorReading.HIGH;
    }

    public bool ReadSecondSensor()
    {
        return _simulator.MovementSensorInput1.Read(_simulator.TimeElapsed) == SensorReading.HIGH;
    }

    public void SetSensorIndicatorPin(bool sensorReportingMotion)
    {
    }

    public bool IsOncePerSecondEvent()
    {
        return false;
    }

    public void Update(ulong currentTime)
    {
    }

    public void SetPWMOnPin9(UInt16 frequency)
    {
    }

    public void SetPWMOnPin10(UInt16 frequency)
    {
    }
    #endregion

}