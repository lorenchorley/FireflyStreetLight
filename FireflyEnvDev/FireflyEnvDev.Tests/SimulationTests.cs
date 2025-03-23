using FireflyEnvDev.Charts;
using FireflyEnvDev.Simulators;
using FireflyEnvDev.Simulators.Interfaces;
using GraphGenerators;
using Microsoft.FSharp.Collections;
using Plotly.NET;

namespace FireflyEnvDev.Tests;

[TestClass]
public sealed class SimulationTests
{
    [TestMethod]
    public void SimulationTest()
    {
        // Arrange
        // =======
        ISimulator sim = new Trivial(TimeSpan.FromMilliseconds(2));

        // Load sensor events
        sim.MovementSensorInput1.SetEvents(
            TimeSpan.FromSeconds(1),
            TimeSpan.FromSeconds(4)
        );
        sim.MovementSensorInput1.SetEvents(
            TimeSpan.FromSeconds(2),
            TimeSpan.FromSeconds(5)
        );


        // Act
        // ===
        sim.Start();

        while (sim.TimeElapsed < TimeSpan.FromSeconds(10))
        {
            sim.Loop();
            sim.TimeElapsed += sim.TickDuration;
        }


        // Assert
        // ======
        PlotGenerator chartGenerator = new PlotGenerator();
        var charts = chartGenerator.Generate(sim.GraphDataOutput);

        var page = ChartGenerator.ArrangeInColumn(ListModule.OfSeq(charts));

        page.SaveHtml("graph.html");
        
    }
}
