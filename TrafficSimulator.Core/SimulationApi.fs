namespace TrafficSimulator.Core
  
open Microsoft.FSharp.Data.UnitSystems.SI.UnitSymbols   
open BaseTypes  
open TrafficSimulator.Core.DomainModel


module Api =
    type Command = Update of TimeInterval
    type Query = Init

    type CommandsHandler = Command -> SimulationState -> SimulationState
    type QueryRunner = Query -> SimulationState

    let handleCommand:CommandsHandler = fun command simulationState -> 
        match command with 
            | Update timeInterval -> Setup.ApiFunctions.updateSimulationState simulationState timeInterval
    let runQuery:QueryRunner = fun query ->
        match query with
            | Init -> Setup.ApiFunctions.init ()
    let locationToPosition = DomainFunctions.locationToPosition // to refactor
        