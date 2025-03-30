using System.Text;

namespace FireflyEnvDev.Outputs;

internal class Serial
{
    private StringBuilder sb { get; } = new ();

    public void print(object s)
    {
        sb.Append(s.ToString());
    }
    
    public void println(object s)
    {
        sb.AppendLine(s.ToString());
    }

    public string GetAllOutput()
    {
        return sb.ToString();
    }
}
