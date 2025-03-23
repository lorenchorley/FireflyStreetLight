namespace FireflyEnvDev.Outputs;

internal class GraphDataOutput
{
    public List<TimeSpan> TimeData { get; } = new();
    private Dictionary<string, List<double>> _timeSeriesDataBySeriesName = new();

    internal IEnumerable<(string, List<double>)> GetTimeSeriesData()
    {
        return _timeSeriesDataBySeriesName.Select(s => (s.Key, s.Value));
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
