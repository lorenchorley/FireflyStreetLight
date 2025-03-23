using FireflyEnvDev.Outputs;
using Plotly.NET;
using Plotly.NET.LayoutObjects;
using System.Data;

namespace FireflyEnvDev.Charts;

public class PlotGenerator
{
    internal IEnumerable<GenericChart> Generate(GraphDataOutput data)
    {
        var timeData = data.TimeData.Select(DateTime.MinValue.Add).ToList();

        var charts =
            data.GetTimeSeriesData()
                .Select(x => Chart2D.Chart.Line<DateTime, double, string>(timeData, x.Item2, Name: x.Item1));

        return charts;
    }
}
