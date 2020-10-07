namespace TrafficSimulator.Core
  
open Microsoft.FSharp.Data.UnitSystems.SI.UnitSymbols   
open BaseTypes  
open TrafficSimulator.Core.DomainModel


module Api =
    type Command = Update of TimeInterval
 
    type CommandsHandler = Command -> SimulationState -> SimulationState
    type Query<'TInput,'TOutput> = 'TInput -> SimulationState option -> 'TOutput    
    //type QueryRunner<'TInput,'TOutput> = 'TInput -> Query<'TInput,'TOutput> -> 'TOutput
   
  //  type Queries = Init of Query<unit,SimulationState> | GetVehicleLocation of Query<Vehicle,Position2d

    let init:Query<unit,SimulationState> = fun () -> fun s -> Setup.ApiFunctions.init()
    let getVehicleLocation:Query<Vehicle,Position2d> = fun v ->  fun s -> DomainFunctions.locationToPosition s.ConnectionsGraph v.Location

    let runQuery (query:Query<'TInput,'TOutput>) input simulationState = query input simulationState
   
    let handleCommand:CommandsHandler = fun command simulationState -> 
        match command with 
            | Update timeInterval -> Setup.ApiFunctions.updateSimulationState simulationState timeInterval
    
    let runQuery:QueryRunner = fun query ->
        match query with
            | Init -> Setup.ApiFunctions.init ()
    let locationToPosition = DomainFunctions.locationToPosition // to refactor
        