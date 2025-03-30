using System.Text.RegularExpressions;

namespace FireflyEnvDev.Arduino;

internal class Transpiler
{
    private string DevelopmentClassHeader = """
        using FireflyEnvDev.Simulators.General;
        using FireflyEnvDev.Outputs;

        namespace FireflyEnvDev.Simulators.FromArduino;
        
        internal class {0}
        {{
            private readonly Functions _f;
            private MapDelegate map {{ get => _f.map; }}
            private ConstrainDelegate constrain {{ get => _f.constrain; }}
            private SinDelegate sin {{ get => _f.sin; }}
            private CosDelegate cos {{ get => _f.cos; }}
            private Serial Serial {{ get => _f.Serial; }}

        """;
    
    private string DevelopmentClassFooter = """
        }
        """;

    public void UpdateDevelopmentClass(FileInfo productionClassFile, FileInfo developmentClassFile)
    {
        string productionClassName = Path.GetFileNameWithoutExtension(productionClassFile.Name);

        IEnumerable<string> lines = File.ReadAllLines(productionClassFile.FullName);

        // Skip initial lines
        lines = lines.SkipWhile(IsDirective)
                     .SkipWhile(string.IsNullOrWhiteSpace);

        lines = lines.Where(l => !IsDirective(l))
                     .Select(TransformDataTypes)
                     .Select(TransformDataLiterals)
                     .Select(ReplaceStateEnumValue)
                     .Select(RemoveClassNameFromMethodSignatures(productionClassName));

        lines = lines.Select(l => $"\t{l}");

        IEnumerable<string> final = 
            Enumerable.Empty<string>()
                .Append(string.Format(DevelopmentClassHeader, productionClassName, productionClassName))
                .Concat(lines)
                .Append(DevelopmentClassFooter);

        File.WriteAllLines(developmentClassFile.FullName, final);
    }

    private bool IsDirective(string line)
    {
        return Regex.IsMatch(line, @"^\s*#");
    }

    private string TransformDataTypes(string line)
    {
        line = Regex.Replace(line, @"\bunsigned long\b", "ulong");
        line = Regex.Replace(line, @"\buint16_t\b", "UInt16");
        line = Regex.Replace(line, @"\bFunctions&", "Functions");
        line = Regex.Replace(line, @"LinkedList<", "CustomLinkedList<");

        return line;
    }
    
    private string TransformDataLiterals(string line)
    {
        line = Regex.Replace(line, @"\b(\d+\.\d+)\b", m => $"{m.Groups[0].Value}f");
        //line = Regex.Replace(line, @"const \b", "");

        return line;
    }
    
    private string ReplaceStateEnumValue(string line)
    {
        line = Regex.Replace(line, @"(?<=\b = )(IDLE|ACCELERATING|HOLDING|DECELERATING)(?=;)", m => $"State.{m.Groups[0].Value}");
        line = Regex.Replace(line, @"(?<=case\s*)(IDLE|ACCELERATING|HOLDING|DECELERATING)(?=\s*:)", m => $"State.{m.Groups[0].Value}");

        return line;
    }
    
    private Func<string, string> RemoveClassNameFromMethodSignatures(string productionClassName)
    => (string line) =>
    {
        line = Regex.Replace(line, $@"(?<type>\w+) {productionClassName}::\b", m => $"public {m.Groups["type"].Value} "); // Methods
        line = Regex.Replace(line, $@"{productionClassName}::\b", m => $"public "); // Constructor

        return line;
    };




    public void UpdateProductionClass(FileInfo developmentClassFile, FileInfo productionClassFile)
    {

    }
}
