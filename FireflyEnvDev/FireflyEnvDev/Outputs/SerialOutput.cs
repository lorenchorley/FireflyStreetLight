using System.Text;

namespace FireflyEnvDev.Outputs;

internal class SerialOutput
{
    private StringBuilder sb { get; } = new ();

    public void Write(string s)
    {
        sb.Append(s);
    }

    public void WriteLine(string s)
    {
        sb.AppendLine(s);
    }

    public string GetAllOutput()
    {
        return sb.ToString();
    }
}
