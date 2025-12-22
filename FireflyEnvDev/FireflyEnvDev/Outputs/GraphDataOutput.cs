using System.Text.RegularExpressions;

namespace FireflyEnvDev.Outputs;

internal class GraphDataOutput
{
    private Serial _serialOutput;
    public List<TimeSpan> TimeData { get; } = new();
    private Dictionary<string, List<double>> _timeSeriesDataBySeriesName = new();
    private Regex _serialOutRegex = new Regex(@"Time:(?<Time>[0-9]+),Sensor:(?<Sensor>[0-9\-\.E]+|NaN),targetFrequency:(?<targetFrequency>[0-9\-\.E]+|NaN),Frequency:(?<Frequency>[0-9\-\.E]+|NaN),Yellow:(?<Yellow>[0-9\-\.E]+),Red:(?<Red>[0-9\-\.E]+),", RegexOptions.Compiled);

    public GraphDataOutput(Serial serialOutput)
    {
        _serialOutput = serialOutput;
    }

    internal IEnumerable<(string, List<DateTime>, List<double>)> GetTimeSeriesData()
    {
        foreach (var str in _serialOutput.GetAllOutput())
        {
            var data = _serialOutRegex.Match(str);

            var time = long.Parse(data.Groups["Time"].Value);
            var sensor = double.Parse(data.Groups["Sensor"].Value);
            var targetFrequency = double.Parse(data.Groups["targetFrequency"].Value);
            var frequency = double.Parse(data.Groups["Frequency"].Value);
            var yellow = double.Parse(data.Groups["Yellow"].Value);
            var red = double.Parse(data.Groups["Red"].Value);

            WriteTimeSeriesDataPoint(
                TimeSpan.FromMilliseconds(time),
                [
                    ("Sensor", sensor), 
                    ("targetFrequency", targetFrequency), 
                    ("Frequency", frequency), 
                    ("Yellow", yellow), 
                    ("Red", red)
                ]
            );
        }

        List<DateTime> dateTimes = TimeData.Select(DateTime.MinValue.Add).ToList();
        return _timeSeriesDataBySeriesName.Select(s => (s.Key, dateTimes, s.Value));
    }

    internal void RegisterTimeSeriesDataNames(params string[] seriesNames)
    {
        foreach (string seriesName in seriesNames)
        {
            _timeSeriesDataBySeriesName.Add(seriesName, new());
        }
    }

    internal void WriteTimeSeriesDataPoint(TimeSpan timeElapsed, params (string, double)[] values)
    {
        if (values.Length != _timeSeriesDataBySeriesName.Count)
            throw new Exception("List lengths do not match, must set all time series data at once");

        TimeData.Add(timeElapsed);

        foreach (var dataPointForSeries in values)
        {
            if (!_timeSeriesDataBySeriesName.TryGetValue(dataPointForSeries.Item1, out var dataList))
            {
                throw new Exception($"Time series data with name {dataPointForSeries.Item1} not registered");
            }

            dataList.Add(dataPointForSeries.Item2);
        }
    }
}
