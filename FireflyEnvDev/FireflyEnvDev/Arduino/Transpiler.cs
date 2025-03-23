using Plotly.NET;
using System.Text.RegularExpressions;

namespace FireflyEnvDev.Arduino;

internal class Transpiler
{
    private string DevelopmentClassHeader = """
        namespace FireflyEnvDev.Simulators.FromArduino;
        
        internal class {0}
        {{
            private Functions _f;

            public {1}(Functions f) {{
                _f = f;
            }}

        """;
    
    private string DevelopmentClassFooter = """
        }
        """;

    public void UpdateDevelopmentClass(FileInfo productionClassFile, FileInfo developmentClassFile)
    {
        string productionClassName = Path.GetFileNameWithoutExtension(productionClassFile.Name);

        IEnumerable<string> lines = File.ReadAllLines(productionClassFile.FullName);

        // Skip initial lines
        lines = lines.SkipWhile(l => l.StartsWith('#') || string.IsNullOrWhiteSpace(l));

        lines = lines.Select(TransformDataTypes)
                     .Select(TransformDataLiterals)
                     .Select(RemoveClassNameFromMethodSignatures(productionClassName));

        lines = lines.Select(l => $"\t{l}");

        IEnumerable<string> final = 
            Enumerable.Empty<string>()
                .Append(string.Format(DevelopmentClassHeader, productionClassName, productionClassName))
                .Concat(lines)
                .Append(DevelopmentClassFooter);

        File.WriteAllLines(developmentClassFile.FullName, final);
    }

    private string TransformDataTypes(string line)
    {
        line = Regex.Replace(line, @"\bunsigned long\b", "ulong");
        //line = Regex.Replace(line, @"const \b", "");

        return line;
    }
    
    private string TransformDataLiterals(string line)
    {
        line = Regex.Replace(line, @"\b(\d+\.\d+)\b", m => $"{m.Groups[0].Value}f");
        //line = Regex.Replace(line, @"const \b", "");

        return line;
    }

    private Func<string, string> RemoveClassNameFromMethodSignatures(string productionClassName)
    => (string line) =>
    {
        line = Regex.Replace(line, $@"{productionClassName}::\b", "");

        return line;
    };




    public void UpdateProductionClass(FileInfo developmentClassFile, FileInfo productionClassFile)
    {

    }
}
