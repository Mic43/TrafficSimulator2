namespace TrafficSimulator.Core

open Microsoft.FSharp.Data.UnitSystems.SI.UnitSymbols     
open DomainModel
open DomainFunctions

module Setup = 
    
    let private distancePerUnit = 10.0<m>
    
    let nextConnectionChooserFactory (vehicle:Vehicle) connectionsGraph = 
                    match vehicle.DrivePath with 
                        | Some (schedule) -> NextConnectionChoosers.chooseByDrivePath schedule
                        | None -> NextConnectionChoosers.chooseRandom connectionsGraph

    let private createVehicleUpdater3 = 
        
        let driverViewDistance = 5.0<m>
        let driverTargetLowSpeed = 5.0<m/s>
        let connectionLenghtProvider  = LenghtProviders.lenghtProvider distancePerUnit        

        //let locationUpdater connectionsGraph timeChange speed location = 
        //    let distanceTravelled = calculateDistanceTravelled speed timeChange
        //    VehicleLocationUpdaters.locationUpdater (connectionLenghtProvider connectionsGraph) (nextConnectionChooser connectionsGraph) distanceTravelled location
       
        let locationUpdater vehicle connectionsGraph timeChange speed location = 
                 let distanceTravelled = calculateDistanceTravelled speed timeChange
                 let nextConnectionChooser = (nextConnectionChooserFactory vehicle connectionsGraph) 
                 let connectionLenghtProvider = (connectionLenghtProvider connectionsGraph)

                 VehicleLocationUpdaters.locationUpdater connectionLenghtProvider nextConnectionChooser distanceTravelled location

        let vehicleUpdater connectionsGraph timeChange vehicle = 
            VehicleUpdaters.simpleVehicleUpdater (locationUpdater vehicle connectionsGraph timeChange vehicle.CurrentMotionParams.Speed) vehicle
        let speedUpd connectionsGraph = VehicleUpdaters.vehicleSpeedUpdater (vehicleUpdater connectionsGraph)
        let accelUpd connectionsGraph = VehicleUpdaters.vehicleAccelerationUpdater driverViewDistance driverTargetLowSpeed 
                                            (connectionLenghtProvider connectionsGraph) (speedUpd connectionsGraph)
        let innerVehiclesUpdater connectionsGraph timeChange vehicles = 
            DomainFunctions.VehiclesUpdaters.update (accelUpd connectionsGraph) timeChange vehicles
        let decoratedVehiclesUpdater connectionsGraph timeChange vehicles = 
            DomainFunctions.VehiclesUpdaters.updateByPlacing (innerVehiclesUpdater connectionsGraph) timeChange vehicles
        decoratedVehiclesUpdater
    let private createVehicleUpdater  = 
      
        let nextConnectionChooser connectionsGraph = NextConnectionChoosers.chooseRandom connectionsGraph

        let connectionLenghtProvider connectionsGraph = LenghtProviders.lenghtProvider distancePerUnit connectionsGraph        
        let locationUpdater connectionsGraph timeChange speed location = 
            let distanceTravelled = calculateDistanceTravelled speed timeChange
            VehicleLocationUpdaters.locationUpdater (connectionLenghtProvider connectionsGraph) (nextConnectionChooser connectionsGraph) distanceTravelled location

        let vehicleUpdater connectionsGraph timeChange vehicle = 
            VehicleUpdaters.simpleVehicleUpdater (locationUpdater connectionsGraph timeChange vehicle.CurrentMotionParams.Speed) vehicle
        let innerVehiclesUpdater connectionsGraph timeChange vehicles = 
            DomainFunctions.VehiclesUpdaters.update (vehicleUpdater connectionsGraph) timeChange vehicles
        let decoratedVehiclesUpdater connectionsGraph timeChange vehicles = 
            DomainFunctions.VehiclesUpdaters.updateByPlacing (innerVehiclesUpdater connectionsGraph) timeChange vehicles
        decoratedVehiclesUpdater

    let private createVehicleUpdater2  = 
            
        let connectionLenghtProviderFactory connectionsGraph = LenghtProviders.lenghtProvider distancePerUnit connectionsGraph        
        let locationUpdater vehicle connectionsGraph timeChange speed location = 
            let distanceTravelled = calculateDistanceTravelled speed timeChange
            let nextConnectionChooser = (nextConnectionChooserFactory vehicle connectionsGraph) 
            let connectionLenghtProvider = (connectionLenghtProviderFactory connectionsGraph)

            VehicleLocationUpdaters.locationUpdater connectionLenghtProvider nextConnectionChooser distanceTravelled location

        let vehicleUpdater connectionsGraph timeChange vehicle = VehicleUpdaters.simpleVehicleUpdater (locationUpdater vehicle connectionsGraph timeChange vehicle.CurrentMotionParams.Speed) vehicle
        let innerVehiclesUpdater connectionsGraph timeChange vehicles = DomainFunctions.VehiclesUpdaters.update (vehicleUpdater connectionsGraph) timeChange vehicles
        let decoratedVehiclesUpdater connectionsGraph timeChange vehicles = DomainFunctions.VehiclesUpdaters.updateByPlacing (innerVehiclesUpdater connectionsGraph) timeChange vehicles
        decoratedVehiclesUpdater

    module ApiFunctions = 
        let updateSimulationState simulationState timeInterval =         
            let vehicleUpdater = createVehicleUpdater3
            let newVehicles = vehicleUpdater simulationState.ConnectionsGraph timeInterval simulationState.Vehicles
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

                ]    
            let connectionsGraph = ConnectionsGraph.create crossings connections 

            match connectionsGraph with
                | Some (cg) -> 
                    let vehicles = [   
                                    {CurrentMotionParams = {Speed = 10.0<m/s>;Acceleration=0.0<m/(s*s)>};
                                    VehicleTypeParams = VehicleTypes.type1; 
                                    Vehicle.Location = 
                                        {VehicleLocation.CurrentProgress = (Fraction.zero);Placing = connections.[0]}; 
                                    DrivePath = 
                                        seq {
                                            connections.[0]
                                            connections.[1]
                                            connections.[2]                                          
                                            } 
                                        |> (DrivePath.create cg) |> Option.orElseWith (fun _ -> failwith "Error creating drive path") 
                                    };
                                    {CurrentMotionParams = {Speed = 10.0<m/s>;Acceleration=0.0<m/(s*s)>};
                                    VehicleTypeParams = VehicleTypes.type1; 
                                    Vehicle.Location = 
                                        {VehicleLocation.CurrentProgress = (Fraction.zero);Placing = connections.[1]};
                                    DrivePath = None
                                    } 
                                    ]
                    {SimulationState.Vehicles = vehicles; SimulationState.ConnectionsGraph = cg}
                | None -> invalidOp "Critical error creating connections graph"

       
            
        
        