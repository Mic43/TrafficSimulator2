namespace TrafficSimulator.Core

open Microsoft.FSharp.Data.UnitSystems.SI.UnitSymbols     
open System
open DomainModel
open DomainFunctions

module Vehicles =    
    type VehicleUpdater = TimeInterval->Vehicle->Vehicle
    type VehicleLocationUpdater = VehicleLocation->VehicleLocation
    type VehiclesUpdater = TimeInterval->Vehicle seq->Vehicle seq

    module UpdateWorkflow = 
        type T = seq<VehicleUpdater>
        let apply (timeInterval:TimeInterval) (vehicle:Vehicle) (workflow: T) =
            workflow |> Seq.fold (fun vehicle vUpdater -> vUpdater timeInterval vehicle) vehicle  


    module ObstacleFinders =       
        type ObstacleFinder = Vehicle->(Obstacle * Distance) option          

        let nearestVehiceAheadOnSameRoad (connectionLenghtProvider:ConnectionLenghtProvider) allVehicles vehicle =
            let currentProgress vehicle = vehicle.Location.CurrentProgress
            let vehicles = allVehicles 
                            |> getVehiclesOnConnection vehicle.Location.Placing 
                            |> Seq.filter (fun v -> not (v.Id = vehicle.Id)) 
                            |> Seq.filter (fun v-> v.Location.CurrentProgress > vehicle.Location.CurrentProgress)
                            |> Seq.map (fun v -> (v,connectionLenghtProvider v.Location.Placing))
                            |> Seq.map (fun (v,connLen) -> (v,Fraction.distanceOnSameConnecton (currentProgress vehicle) (currentProgress v) connLen))   
            if vehicles |> Seq.isEmpty 
                then None
            else 
                match vehicles |>  Seq.minBy (fun (v,distance) -> distance) with
                | (vehicle,distance) -> Some (Obstacle.OtherVehicle vehicle,distance) 
             
        let nextCrossing (connectionGraph:ConnectionsGraph) (connectionLenghtProvider:ConnectionLenghtProvider) (vehicle:Vehicle) =
              let connectionLen = connectionLenghtProvider vehicle.Location.Placing
              Some ( Obstacle.Crossing (vehicle.Location.Placing.EndId |> ConnectionsGraph.crossing connectionGraph)
                    ,Fraction.distanceOnSameConnecton vehicle.Location.CurrentProgress Fraction.one connectionLen)

        let nearestObstacle (finders:ObstacleFinder seq) vehicle =             
            let obstacles = finders |> Seq.map (fun finder -> finder vehicle) |> Seq.collect (fun obstalce_distance  -> Option.toList obstalce_distance) 
            if obstacles |> Seq.isEmpty 
                then None
            else 
                Some(obstacles |> Seq.minBy (fun (_,distance) -> distance))
           
    module Updaters =
      
        let compositeUpdater (vehicleUpdaters:VehicleUpdater seq) (timeChange:TimeInterval) (vehicle:Vehicle) = 
            vehicleUpdaters |> Seq.fold (fun vehicle updater -> updater timeChange vehicle) vehicle
        let breakOnlyIfSpeedGreaterThan minSpeed (vehicleUpdater:VehicleUpdater) (timeChange:TimeInterval) (vehicle:Vehicle)  = 
            if vehicle.CurrentMotionParams.Speed < minSpeed then vehicle
            else vehicleUpdater timeChange vehicle
        let breakByDistance (map: VehicleDistanceAccelerationMap) (obstacleFinder: ObstacleFinders.ObstacleFinder) 
                                  (timeChange:TimeInterval) (vehicle:Vehicle) = 
            match (obstacleFinder vehicle) with 
                | Some (_,obstacleDistance) -> 
                    if obstacleDistance < 0.0<m>
                        then vehicle
                    else
                        let newAccel = match (map |> Map.toList |> List.tryFind (fun (distance,_) -> obstacleDistance < distance)) with 
                                                    | Some (_,accel) -> accel
                                                    | None -> vehicle.CurrentMotionParams.Acceleration
                        let newMotionParams = {vehicle.CurrentMotionParams with Acceleration = newAccel}
                        {vehicle with CurrentMotionParams = newMotionParams}        
                | None -> vehicle       
        let accelerateMaximally  (timeChange:TimeInterval) (vehicle:Vehicle) = 
            let newMotionParams = {vehicle.CurrentMotionParams with Acceleration = vehicle.VehicleTypeParams.MaximumParamaters.Acceleration}
            {vehicle with CurrentMotionParams = newMotionParams}  

        let accelerationUpdater viewDistnace targetSpeed (connectionLenghtProvider:ConnectionLenghtProvider) 
                                (timeChange:TimeInterval) (vehicle:Vehicle) = 
            let len = connectionLenghtProvider vehicle.Location.Placing
            let travelled = Fraction.toDistance len vehicle.Location.CurrentProgress
            //  let targetSpeed = 1.0<m/s>
            let newAccel = 
                if len - travelled < viewDistnace
                    then
                        if vehicle.CurrentMotionParams.Speed > targetSpeed
                            then vehicle.VehicleTypeParams.MinimumParameters.Acceleration 
                            else 0.0<m/(s*s)>
                    else vehicle.VehicleTypeParams.MaximumParamaters.Acceleration
            let newMotionParams = {vehicle.CurrentMotionParams with Acceleration = newAccel}
            {vehicle with CurrentMotionParams = newMotionParams}        

        let speedUpdater (timeChange:TimeInterval) vehicle = 
            let (TimeInterval dt) = timeChange
            let newSpeed =  vehicle.CurrentMotionParams.Speed + (vehicle.CurrentMotionParams.Acceleration * dt) 
            let maxSpeed = vehicle.VehicleTypeParams.MaximumParamaters.Speed
            let minSpeed = vehicle.VehicleTypeParams.MinimumParameters.Speed

            let speedCapped = if newSpeed > minSpeed then (if newSpeed < maxSpeed then newSpeed else maxSpeed) else minSpeed
            let newMotionParams = {vehicle.CurrentMotionParams with Speed = speedCapped}
            {vehicle with CurrentMotionParams = newMotionParams}

        let locationUpdater (conectionLenghtProvider:ConnectionLenghtProvider) (nextConnectionChooser:NextConnectionChooser) 
                                   (timeChange:TimeInterval) (vehicle:Vehicle) =          
            let distanceTravelled = calculateDistanceTravelled vehicle.CurrentMotionParams.Speed timeChange
            let rec locationUpdaterHelper conectionLenghtProvider (nextConnectionChooser:NextConnectionChooser) currentConnection distance initialDistance =             
                let locationUpdaterHelper = locationUpdaterHelper conectionLenghtProvider (nextConnectionChooser:NextConnectionChooser) 
                let lenght = conectionLenghtProvider currentConnection
                let calculateFraction () = let v = Fraction.fromDistance (initialDistance + distance) lenght 
                                           v |> Option.defaultWith (fun () -> failwith "Cannot create fraction greater than 1: ")
                let buildResult () =  {VehicleLocation.CurrentProgress = calculateFraction();VehicleLocation.Placing = currentConnection}

                if initialDistance + distance <= lenght 
                    then                     
                        buildResult()
                    else
                        let nextConnection = nextConnectionChooser currentConnection.EndId
                        match nextConnection with 
                            | None -> {VehicleLocation.CurrentProgress = Fraction.one;VehicleLocation.Placing = currentConnection}                       
                            | Some(nextConnection) -> 
                                let distanceLeft = (distance - (lenght - initialDistance))
                                locationUpdaterHelper nextConnection distanceLeft 0.0<m>
            let currentConnection = vehicle.Location.Placing
            let distanceFromStart = vehicle.Location.CurrentProgress |> Fraction.toDistance (conectionLenghtProvider currentConnection)             
            let newLocation = locationUpdaterHelper conectionLenghtProvider nextConnectionChooser currentConnection distanceTravelled distanceFromStart
            {vehicle with Location = newLocation}
    
    type VehicleWithWorflow = {Vehicle:Vehicle; Workflow: UpdateWorkflow.T }

    module SequenceUpdaters =          
        type T = VehicleWithWorflow seq->TimeInterval->Vehicle seq
      
        let update (workflows:VehicleWithWorflow seq) (timeChange:TimeInterval) = 
                    workflows |> Seq.map (fun w -> UpdateWorkflow.apply timeChange w.Vehicle w.Workflow)                                                

        let updateByPlacing (sequenceUpdater:T) (workflows:VehicleWithWorflow seq) (timeChange:TimeInterval) = 
                workflows |> Seq.groupBy (fun w -> w.Vehicle.Location.Placing) 
                          |> Seq.map ( (fun (_,vehicles) ->  vehicles |> Seq.sortByDescending (fun w->w.Vehicle.Location.CurrentProgress)) 
                                        >> (fun vehiclesOnSameConnection -> timeChange |> sequenceUpdater vehiclesOnSameConnection))
                          |> Seq.fold (fun acc cur -> acc |> Seq.append cur) Seq.empty
