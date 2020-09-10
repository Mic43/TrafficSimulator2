namespace TrafficSimulator.Core

open Microsoft.FSharp.Data.UnitSystems.SI.UnitSymbols     
open DomainModel
open DomainFunctions
open Vehicles
module Setup = 
    
    let private distancePerUnit = 10.0<m>
    
    let private nextConnectionChooserFactory (vehicle:Vehicle) connectionsGraph = 
                    match vehicle.DrivePath with 
                        | Some (schedule) -> NextConnectionChoosers.chooseByDrivePath schedule
                        | None -> NextConnectionChoosers.chooseRandom connectionsGraph

    let private buildAccelerationUpdater connectionsGraph connectionLenghtProvider allVehicles = 
        let vehiclleAccelMap = Map.empty // .Add( 10.0<m>,-2.0<m/(s*s)>)
                                        //.Add( 5.0<m>,-10.0<m/(s*s)>)
                                        //.Add( 3.0<m>,-18.0<m/(s*s)>)
                                        .Add( 2.0<m>,-20.0<m/(s*s)>)
                                        .Add( 1.0<m>,-50.0<m/(s*s)>)


        let finders = seq {
            ObstacleFinders.nearestVehiceAheadOnSameRoad connectionLenghtProvider allVehicles
            ObstacleFinders.nextCrossing connectionsGraph connectionLenghtProvider
        }
        let finder = ObstacleFinders.nearestObstacle finders                
        let updaters = seq {
            Updaters.accelerateMaximally;
            Updaters.breakByDistance vehiclleAccelMap finder
        }
        Updaters.compositeUpdater updaters
    let private buildUpdateWorkflowForVehicle allvehicles connectionsGraph vehicle : UpdateWorkflow.T =       
        let driverViewDistance = 8.0<m>
        let driverTargetLowSpeed = 1.0<m/s>

        let connectionLenghtProvider  = LenghtProviders.lenghtProvider distancePerUnit connectionsGraph       
        let nextConnectionChooser = (nextConnectionChooserFactory vehicle connectionsGraph) 

        //let accelerationUpdater = Updaters.accelerationUpdater driverViewDistance driverTargetLowSpeed connectionLenghtProvider
        let accelerationUpdater =  buildAccelerationUpdater connectionsGraph connectionLenghtProvider allvehicles
        let speedUpdater = Updaters.speedUpdater
        let locationUpdater = Updaters.locationUpdater connectionLenghtProvider (nextConnectionChooser)

        seq {accelerationUpdater;speedUpdater;locationUpdater}

    let private buildSequenceUpdater connectionsGraph vehicles =       
        let workflows = vehicles  |> Seq.map (fun vehicle -> {Vehicle = vehicle;Workflow = buildUpdateWorkflowForVehicle vehicles connectionsGraph vehicle})
        let innerVehiclesUpdater = SequenceUpdaters.update
        let decoratedVehiclesUpdater = SequenceUpdaters.updateByPlacing innerVehiclesUpdater workflows
        decoratedVehiclesUpdater

    module ApiFunctions =         
        let updateSimulationState simulationState timeInterval =                
            let vehicleSequenceUpdater = buildSequenceUpdater simulationState.ConnectionsGraph simulationState.Vehicles           
            let newVehicles = vehicleSequenceUpdater timeInterval
            { simulationState with Vehicles = newVehicles |> Seq.toList }
        let init () = 
            let crossings =
                Map.empty (* Start with empty Map *)
                   .Add( CrossingId 1, {Name = Some "aaa";Position = Position2d {X = 1.0;Y =2.0}})
                   .Add( CrossingId 2, {Name = None ;Position = Position2d {X = 1.5;Y =4.0}})
                   .Add( CrossingId 3, {Name = None ;Position = Position2d {X = 5.5;Y =7.0}})
                   .Add( CrossingId 4, {Name = None ;Position = Position2d {X = 3.5;Y = 0.5}})

            let connections = [
                {ConnectionType = QuadraticBezier (Position2d {X=0.5;Y=2.5});StartId= CrossingId 1;EndId = CrossingId 2}
                {ConnectionType = Linear ;StartId= CrossingId 2;EndId = CrossingId 1}
                {ConnectionType = Linear ;StartId= CrossingId 1;EndId = CrossingId 3}
                {ConnectionType = Linear ;StartId= CrossingId 3;EndId = CrossingId 1}
                {ConnectionType = QuadraticBezier (Position2d {X=1.5;Y= 1.0});StartId= CrossingId 1;EndId = CrossingId 4}
                {ConnectionType = QuadraticBezier (Position2d {X=1.5;Y= 1.0});StartId= CrossingId 4;EndId = CrossingId 1}

                ]    
            let connectionsGraph = ConnectionsGraph.create crossings connections 

            match connectionsGraph with
                | Some (cg) -> 
                    let vehicles = [{   Id = VehicleId 1
                                        CurrentMotionParams = {Speed = 5.0<m/s>;Acceleration=0.0<m/(s*s)>};
                                        VehicleTypeParams = VehicleTypes.type1; 
                                        Vehicle.Location = 
                                            {VehicleLocation.CurrentProgress = (Fraction.create 0.5) |> Option.get ;Placing = connections.[0]}; 
                                        DrivePath = None                                       
                                    };
                                    {   
                                        Id = VehicleId 2
                                        CurrentMotionParams = {Speed = 6.0<m/s>;Acceleration=0.0<m/(s*s)>};
                                        VehicleTypeParams = VehicleTypes.type2; 
                                        Vehicle.Location = 
                                            {VehicleLocation.CurrentProgress = (Fraction.zero);Placing = connections.[0]};    
                                        DrivePath = None                                                                                   
                                    }

                                    //DrivePath = 
                                    //                                      seq {
                                    //                                          connections.[0]
                                    //                                          connections.[1]
                                    //                                          connections.[2]                                          
                                    //                                          } 
                                    //                                      |> (DrivePath.create cg) |> Option.orElseWith (fun _ -> failwith "Error creating drive path") 
                                    //{
                                    //    Id = VehicleId 2
                                    //    CurrentMotionParams = {Speed = 10.0<m/s>;Acceleration=0.0<m/(s*s)>};
                                    //    VehicleTypeParams = VehicleTypes.type2; 
                                    //    Vehicle.Location = 
                                    //        {VehicleLocation.CurrentProgress = (Fraction.zero);Placing = connections.[1]};
                                    //    DrivePath = None
                                    //}
                                    ]
                    {SimulationState.Vehicles = vehicles; SimulationState.ConnectionsGraph = cg}
                | None -> invalidOp "Critical error creating connections graph"

       
            
        
        