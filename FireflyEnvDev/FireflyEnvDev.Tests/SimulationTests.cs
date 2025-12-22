using FireflyEnvDev.Charts;
using FireflyEnvDev.Simulators;
using FireflyEnvDev.Simulators.Interfaces;
using GraphGenerators;
using Microsoft.FSharp.Collections;
using Plotly.NET;
using System.Diagnostics;
using System.Text.RegularExpressions;

namespace FireflyEnvDev.Tests;

[TestClass]
public sealed class SimulationTests
{
    private TimeSpan SimulationDuration = TimeSpan.FromSeconds(10);
    private TimeSpan TickDuration = TimeSpan.FromMilliseconds(2);

    [TestMethod]
    public void TrivialSimulationTest()
    {
        // Arrange
        // =======
        ISimulator sim = new Trivial(TickDuration);

        // Load sensor events
        sim.MovementSensorInput1.SetEvents(
            TimeSpan.FromSeconds(1),
            TimeSpan.FromSeconds(4)
        );
        sim.MovementSensorInput2.SetEvents(
            TimeSpan.FromSeconds(2),
            TimeSpan.FromSeconds(5)
        );


        // Act
        // ===
        sim.Start();

        while (sim.TimeElapsed < SimulationDuration)
        {
            sim.Loop();
            sim.TimeElapsed += sim.TickDuration;
        }


        // Assert
        // ======
        PlotGenerator chartGenerator = new PlotGenerator();
        var charts = chartGenerator.Generate(sim.GraphDataOutput);

        var page = ChartGenerator.ArrangeInColumn(ListModule.OfSeq(charts));

        string path = Path.GetFullPath("graph.html");
        page.SaveHtml(path);

        // Open the graph in the default application
        var p = new Process();
        p.StartInfo = new ProcessStartInfo(path)
        {
            UseShellExecute = true
        };
        p.Start();

    }

    [TestMethod]
    public void V1SimulationTest()
    {
        // Arrange
        // =======
        ISimulator sim = new VerLuisantV1(TickDuration);

        // Load sensor events
        sim.MovementSensorInput1.SetEvents(
            TimeSpan.FromSeconds(1)
            //TimeSpan.FromSeconds(4)
        );
        //sim.MovementSensorInput2.SetEvents(
        //    TimeSpan.FromSeconds(2),
        //    TimeSpan.FromSeconds(5)
        //);


        // Act
        // ===
        sim.Start();

        int loopCount = 0;
        while (sim.TimeElapsed < SimulationDuration)
        {
            loopCount++;
            sim.Loop();
            sim.TimeElapsed += sim.TickDuration;
        }

        Debug.WriteLine($"Finished after {loopCount} cycles");


        // Assert
        // ======

        var serialOutput = sim.SerialOutput.GetAllOutput();

        // Save raw serial text
        string serialPath = Path.GetFullPath("serial.out");
        File.WriteAllText(serialPath, string.Join("\n", serialOutput));

        // Parse serial output into graph data
        
        // Compile graph from parsed serial out text
        PlotGenerator chartGenerator = new PlotGenerator();
        var charts = chartGenerator.Generate(sim.GraphDataOutput);
        var page = ChartGenerator.ArrangeInColumn(ListModule.OfSeq(charts));


        // Save graph
        string graphPath = Path.GetFullPath("graph.html");
        page.SaveHtml(graphPath);

        Debug.WriteLine($"Generated file : {graphPath}");

        // Open the graph in the default application
        var p = new Process();
        p.StartInfo = new ProcessStartInfo(graphPath)
        {
            UseShellExecute = true
        };
        p.Start();

    }

    GenericChart ArrangeInColumn(IEnumerable<GenericChart> charts)
    {
        return ChartGenerator.ArrangeInColumn(ListModule.OfSeq(charts));
    }
}
