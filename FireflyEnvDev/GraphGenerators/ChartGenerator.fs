namespace GraphGenerators

open Plotly.NET

module ChartGenerator =
    let x = [ 1.; 2.; 3.; 4.; 5.; 6.; 7.; 8.; 9.; 10. ]
    let y = [ 2.; 1.5; 5.; 1.5; 3.; 2.5; 2.5; 1.5; 3.5; 1. ]

    let GenerateTimeSeries name data =
        Chart.Point(x = x, y = y, Name = "1,1")
            |> Chart.withXAxisStyle "x1"
            |> Chart.withYAxisStyle "y1"

    let ArrangeInColumn (charts : GenericChart list) =
        charts
        |> Chart.Grid(charts.Length, 1)
        |> Chart.withSize (1400, 400)
