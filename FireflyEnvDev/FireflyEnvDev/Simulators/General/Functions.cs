using FireflyEnvDev.Simulators.Interfaces;

namespace FireflyEnvDev.Simulators.General;

internal class Functions
{
    private readonly ISimulator _simulator;

    public Functions(ISimulator simulator)
    {
        _simulator = simulator;
    }


}