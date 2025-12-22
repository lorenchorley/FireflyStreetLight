using System.Text;

namespace FireflyEnvDev.Outputs;

internal class Serial
{
    private StringBuilder _sb { get; } = new ();
    private List<string> _lines = new();

    public void print(object s)
    {
        _sb.Append(s.ToString());
    }
    
    public void println(object s)
    {
        _sb.Append(s.ToString());
        _lines.Add(_sb.ToString());
        _sb.Clear();
    }
    
    public void print(string s)
    {
        _sb.Append(s);
    }
    
    public void println(string s)
    {
        _sb.Append(s);
        _lines.Add(_sb.ToString());
        _sb.Clear();
    }

    public List<string> GetAllOutput()
    {
        return _lines;
    }
}
