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
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    
    public float constrain(float x, float min, float max)
    {
        if (x < min)
        {
            return min;
        }

        if (x > max)
        {
            return max;
        }

        return x;
    }

    public float sin(float x)
    {
        return MathF.Sin(x);
    }

    public float cos(float x)
    {
        return MathF.Cos(x);
    }
    #endregion

    #region
    public bool ReadFirstSensor()
    {
        return _simulator.MovementSensorInput1.Read(_simulator.TimeElapsed);
    }

    public bool ReadSecondSensor()
    {
        return _simulator.MovementSensorInput1.Read(_simulator.TimeElapsed);
    }

    private TimeSpan lastEvent = TimeSpan.Zero;

    public bool IsOncePerSecondEvent()
    {
        if (_simulator.TimeElapsed - lastEvent >= TimeSpan.FromSeconds(1))
        {
            lastEvent += TimeSpan.FromSeconds(1);
            return true;
        }
        
        return false;
    }

    public void Update(ulong currentTime)
    {
    }

    public void SetSensorIndicatorPin(bool sensorReportingMotion)
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