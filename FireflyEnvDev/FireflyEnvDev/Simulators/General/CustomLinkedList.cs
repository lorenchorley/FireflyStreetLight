
namespace FireflyEnvDev.Simulators.General;

internal class CustomLinkedList<T>
{
    private System.Collections.Generic.LinkedList<T> _ls = new();

    internal void add(T currentTime)
    {
        _ls.AddLast(currentTime);
    }

    internal T get(int v)
    {
        return _ls.ElementAt(v);
    }

    internal void remove(int v)
    {
        _ls.Remove(get(v));
    }

    internal int size()
    {
        return _ls.Count;
    }
}