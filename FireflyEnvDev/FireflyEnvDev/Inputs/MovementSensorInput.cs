using System.Diagnostics;

namespace FireflyEnvDev.Inputs;

internal class MovementSensorInput
{
    private Queue<TimeSpan> _timeSpans = new();

    internal bool Read(TimeSpan timeElapsed)
    {
        bool shouldPop = _timeSpans.Count > 0 && timeElapsed > _timeSpans.Peek();

        if (shouldPop)
        {
            _timeSpans.Dequeue();
            Debug.WriteLine($"Sensor read event after {timeElapsed.TotalMilliseconds}ms");
        }

        return shouldPop;
    }

    internal void SetEvents(params TimeSpan[] timeSpans)
    {
        _timeSpans = new(timeSpans.Order());
    }
}
