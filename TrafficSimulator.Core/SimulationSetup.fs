namespace TrafficSimulator.Core

open Microsoft.FSharp.Data.UnitSystems.SI.UnitSymbols     
open Common
open BaseTypes
open DomainModel
open DomainFunctions
open Vehicles

type SimulationState = {ConnectionsGraph:ConnectionsGraph; Vehicles: Vehicle seq;TrafficLights:TrafficLights.LightSystem Set } // TODO: change to map by id

module Setup = 
    
    let private distancePerUnit = 10.0<m>
    
    let private nextConnectionChooserFactory (vehicle:Vehicle) connectionsGraph = 
                    match vehicle.DrivePath with 
                        | Some (schedule) -> NextConnectionChoosers.chooseByDrivePath schedule
                        | None -> NextConnectionChoosers.chooseRandom connectionsGraph
    let private connectionLenghtProvider connectionsGraph = LenghtProviders.lenghtProvider distancePerUnit connectionsGraph  
    let private buildAccelerationUpdater connectionLenghtProvider simulationState = 
        let vehiclleAccelMapHard = Map.empty                                                                                                                     
                                        .Add( 3.0<m>,-10.0<m/(s*s)>)
                                        .Add( 2.0<m>,-20.0<m/(s*s)>)
                                        .Add( 1.0<m>,-150.0<m/(s*s)>)
                                        .Add( 0.5<m>,-150.0<m/(s*s)>)
        let vehiclleAccelMapLight = Map.empty .Add( 4.0<m>,-2.0<m/(s*s)>)
                                              .Add( 3.0<m>,-8.0<m/(s*s)>)                                              

        //let finders = seq {
        //    ObstacleFinders.nearestVehiceAheadOnSameRoad connectionLenghtProvider allVehicles
        //    ObstacleFinders.nextCrossing connectionsGraph connectionLenghtProvider
        //}
        let vehiclesFinder = ObstacleFinders.nearestVehiceAheadOnSameConnection connectionLenghtProvider simulationState.Vehicles
        let crossingFinder = ObstacleFinders.nextCrossing simulationState.ConnectionsGraph connectionLenghtProvider
        let trafficLightsFinder = ObstacleFinders.nearestRedLightAheadOnSameConnection connectionLenghtProvider simulationState.TrafficLights
        let nearestFinder = ObstacleFinders.nearestObstacle (seq { vehiclesFinder;trafficLightsFinder })

        //let finder = ObstacleFinders.nearestObstacle finders                
        let updaters = seq {
            Updaters.accelerateMaximally;
            Updaters.breakOnlyIfSpeedGreaterThan 5.0<m/s> (Updaters.breakByDistance vehiclleAccelMapLight crossingFinder)            
            Updaters.breakOnlyIfSpeedGreaterThan 0.0<m/s> (Updaters.breakByDistance vehiclleAccelMapHard nearestFinder)          
        }
        Updaters.compositeUpdater updaters
    let private buildUpdateWorkflowForVehicle simulationState vehicle : UpdateWorkflow.T =              
        let connectionsGraph = simulationState.ConnectionsGraph

        let connectionLenghtProvider  = connectionLenghtProvider connectionsGraph       
        let nextConnectionChooser = (nextConnectionChooserFactory vehicle connectionsGraph) 

        //let accelerationUpdater = Updaters.accelerationUpdater driverViewDistance driverTargetLowSpeed connectionLenghtProvider
        let accelerationUpdater =  buildAccelerationUpdater  connectionLenghtProvider simulationState
        let speedUpdater = Updaters.speedUpdater
        let locationUpdater = Updaters.locationUpdater connectionLenghtProvider (nextConnectionChooser)

        seq {accelerationUpdater;speedUpdater;locationUpdater}

    let private buildSequenceUpdater simulationState =       
        let workflows = simulationState.Vehicles  |> Seq.map (fun vehicle -> {Vehicle = vehicle;Workflow = buildUpdateWorkflowForVehicle simulationState vehicle})
        let innerVehiclesUpdater = SequenceUpdaters.update
        let decoratedVehiclesUpdater = SequenceUpdaters.updateByPlacing innerVehiclesUpdater workflows
        decoratedVehiclesUpdater

    module ApiFunctions = 
        open TrafficLights
        let updateSimulationState simulationState timeInterval =                
            let vehicleSequenceUpdater = buildSequenceUpdater simulationState           
            let newVehicles = vehicleSequenceUpdater timeInterval
            let newTrafficLights = simulationState.TrafficLights |> Set.map (fun tl -> tl |> Functions.updateLightSystem timeInterval)
            { simulationState with Vehicles = newVehicles |> Seq.toList ; TrafficLights = newTrafficLights}
        let init () = 
            let crossings =
                Map.empty (* Start with empty Map *)
                   .Add( CrossingId 1, {Name = Some "aaa";Position = Position2d {X = 1.0;Y =2.0}})
                   .Add( CrossingId 2, {Name = None ;Position = Position2d {X = 1.5;Y =4.0}})
                   .Add( CrossingId 3, {Name = None ;Position = Position2d {X = 5.5;Y =4.0}})
                   .Add( CrossingId 4, {Name = None ;Position = Position2d {X = 3.5;Y = 0.5}})
                   .Add( CrossingId 5, {Name = None ;Position = Position2d {X = 0.5;Y = 1.5}})
                   .Add( CrossingId 6, {Name = None ;Position = Position2d {X = 3.0;Y = 0.0}})
            let connections = [
                DomainFunctions.bezierFromCrossings crossings (CrossingId 1) (CrossingId 2) (Position2d {X=0.5;Y=2.5})
                DomainFunctions.bezierFromCrossings crossings (CrossingId 2) (CrossingId 1) (Position2d {X=0.5;Y=2.5})
                {ConnectionType = Linear ;StartId= CrossingId 1;EndId = CrossingId 3}
                {ConnectionType = Linear ;StartId= CrossingId 3;EndId = CrossingId 1}
                DomainFunctions.bezierFromCrossings crossings (CrossingId 1) (CrossingId 4) (Position2d  {X=1.5;Y= 1.0})            
                DomainFunctions.bezierFromCrossings crossings (CrossingId 4) (CrossingId 1) (Position2d  {X=1.5;Y= 1.0})                            
                {ConnectionType = Linear ;StartId= CrossingId 1;EndId = CrossingId 5}
                {ConnectionType = Linear ;StartId= CrossingId 5;EndId = CrossingId 1}
                DomainFunctions.bezierFromCrossings crossings (CrossingId 5) (CrossingId 6) (Position2d  {X=1.5;Y= 0.0})                            
                DomainFunctions.bezierFromCrossings crossings (CrossingId 6) (CrossingId 5) (Position2d  {X=1.5;Y= 0.0})                                            
                ]    
            let connectionsGraph = ConnectionsGraph.create crossings connections 

            let createVehicle  initialSpeed id vehicleType start = 
                {   Id = VehicleId id 
                    CurrentMotionParams = {Speed = initialSpeed;Acceleration=0.0<m/(s*s)>};
                    VehicleTypeParams = vehicleType; 
                    Vehicle.Location = 
                        {ObjectLocation.CurrentProgress = (Fraction.tryCreate start) |> Option.get ;Placing = connections.[1]}; 
                    DrivePath = None                                       
                };
            let count = 10
            let vehicles = Seq.init count (fun i -> createVehicle 0.0<m/s> i (if (i % 2 = 0) then VehicleTypes.type1 else VehicleTypes.type2) ((float i) / (float count))) |> Seq.toList                    

            let greenRedInterval = (TimeInterval 5.0<s>)   
            
            let trafficLights = Functions.createCrossingLightSystem (set [connections.[1];connections.[5]]) (set [connections.[3];connections.[7]]) 
                                                                     greenRedInterval greenRedInterval (connectionLenghtProvider connectionsGraph.Value)
          
            //let trafficLights = TrafficLights.createSimpleLightSystem greenRedInterval greenRedInterval connections.[0] connections.[1] (Fraction.create 0.5).Value 

            (optional {
                let! cG = connectionsGraph
                let! tl = trafficLights
                return {SimulationState.Vehicles = vehicles; SimulationState.ConnectionsGraph = cG;TrafficLights = [tl] |> set}
            }) |> Option.defaultWith (fun () -> failwith "Error creating graph")
          

       
            
        
        