using FireflyEnvDev.Arduino;

if (args.Length < 3)
    throw new ArgumentException("Not enough arguments");

var sourceFile = new FileInfo(Path.GetFullPath(args[1]));
var targetFile = new FileInfo(Path.GetFullPath(args[2]));

bool dev2prod = args.Any(a => string.Equals(a, "-dev2prod"));
bool prod2dev = args.Any(a => string.Equals(a, "-prod2dev"));

if (!dev2prod && !prod2dev)
{
    Console.WriteLine("No action taken");
    return;
}

if (!sourceFile.Exists)
{
    throw new ArgumentException($"Source path {sourceFile.FullName} does not exist");
}

if (targetFile.Exists)
{
    Console.WriteLine($"Deleting target file : {targetFile.FullName}");
    targetFile.Delete();
}

Transpiler transpiler = new();

if (dev2prod)
{
    if (!string.Equals(sourceFile.Extension, ".cs"))
        throw new ArgumentException("Source file is not of type .cs");

    if (!string.Equals(targetFile.Extension, ".cpp"))
        throw new ArgumentException("Source file is not of type .cpp");

    transpiler.UpdateProductionClass(sourceFile, targetFile);

    Console.WriteLine($"Transpiled to {targetFile.FullName}");

    return;
}

if (prod2dev)
{
    if (!string.Equals(sourceFile.Extension, ".cpp"))
        throw new ArgumentException("Source file is not of type .cpp");

    if (!string.Equals(targetFile.Extension, ".cs"))
        throw new ArgumentException("Source file is not of type .cs");

    transpiler.UpdateDevelopmentClass(sourceFile, targetFile);

    Console.WriteLine($"Transpiled to {targetFile.FullName}");

    return;
}