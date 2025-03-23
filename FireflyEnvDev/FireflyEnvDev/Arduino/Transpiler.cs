namespace FireflyEnvDev.Arduino;

internal class Transpiler
{
    public void UpdateDevelopmentClass(FileInfo productionClassFile, FileInfo developmentClassFile)
    {
        var lines = File.ReadAllLines(productionClassFile.FullName);

        lines = lines.SkipWhile();
    }

    public void UpdateProductionClass(FileInfo developmentClassFile, FileInfo productionClassFile)
    {

    }
}
