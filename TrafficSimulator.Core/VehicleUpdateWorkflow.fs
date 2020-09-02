namespace TrafficSimulator.Core

open Microsoft.FSharp.Data.UnitSystems.SI.UnitSymbols     
open System
open DomainModel
open DomainFunctions

module Vehicles =    
    module UpdateWorkflow = 
        type T = seq<VehicleUpdater>
        let apply (timeInterval:TimeInterval) (vehicle:Vehicle) (workflow: T) =
            workflow |> Seq.fold (fun vehicle vUpdater -> vUpdater timeInterval vehicle) vehicle  
    module Updaters = 
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
        type T = TimeInterval->VehicleWithWorflow seq->Vehicle seq
      
        let update (timeChange:TimeInterval) (workflows:VehicleWithWorflow seq) = 
                    workflows |> Seq.map (fun w -> UpdateWorkflow.apply timeChange w.Vehicle w.Workflow)                                                

        let updateByPlacing (sequenceUpdater:T) (timeChange:TimeInterval) (workflows:VehicleWithWorflow seq) = 
                workflows |> Seq.groupBy (fun w -> w.Vehicle.Location.Placing) 
                          |> Seq.map ( (fun (_,vehicles) ->  vehicles |> Seq.sortByDescending (fun w->w.Vehicle.Location.CurrentProgress)) 
                                        >> (fun vehiclesOnSameConnection -> vehiclesOnSameConnection |> sequenceUpdater timeChange))
                          |> Seq.fold (fun acc cur -> acc |> Seq.append cur) Seq.empty
