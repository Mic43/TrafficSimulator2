namespace TrafficSimulator.Core

open Microsoft.FSharp.Data.UnitSystems.SI.UnitSymbols
open FSharp.Collections.ParallelSeq
open Common
open BaseTypes
open DomainModel
open DomainFunctions
open Collisions
open Obstacles

module Vehicles =
    type VehicleUpdater = TimeInterval -> Vehicle -> Vehicle
    type VehicleSequenceUpdater = TimeInterval -> Vehicle seq -> Vehicle seq

    module Updaters =
        let stateUpdater collisionMaxDuration (collisions: Collision Set) (timeChange: TimeInterval) (vehicle: Vehicle) =           
            match vehicle.State with
            | Running -> 
                let flattenSet = collisions
                                    |> Set.fold (fun acc s -> acc |> (Set.union (s.getCollidables()) )) Set.empty
                if flattenSet |> Set.map (function | Vehicle v -> v) |> Set.contains vehicle 
                then {vehicle.makeImmovable() with State = Collided {StateDuration = TimeInterval.zero()}}
                else vehicle
            | Collided c ->
                let newDuration = c.StateDuration + timeChange

                if newDuration > collisionMaxDuration
                then {vehicle with State = ToBeRemoved}
                else  {vehicle with State = Collided {StateDuration = newDuration}}
            | ToBeRemoved -> vehicle
          
        let compositeUpdater (vehicleUpdaters: VehicleUpdater seq) (timeChange: TimeInterval) (vehicle: Vehicle) =
            vehicleUpdaters
            |> Seq.fold (fun vehicle updater -> updater timeChange vehicle) vehicle

        let breakOnlyIfSpeedGreaterThan minSpeed
                                        (vehicleUpdater: VehicleUpdater)
                                        (timeChange: TimeInterval)
                                        (vehicle: Vehicle)
                                        =
            if vehicle.CurrentMotionParams.Speed < minSpeed
            then vehicle
            else vehicleUpdater timeChange vehicle

        let breakByDistance (map: VehicleDistanceAccelerationMap)
                            (obstacleFinder: ObstacleFinder)
                            (timeChange: TimeInterval)
                            (vehicle: Vehicle)
                            =
            match (obstacleFinder vehicle) with
            | Some (_, obstacleDistance) ->
                if obstacleDistance < 0.0<m> then
                    vehicle
                else
                    let newAccel =
                        match (map
                               |> Map.toList
                               |> List.tryFind (fun (distance, _) -> obstacleDistance < distance)) with
                        | Some (_, accel) -> accel
                        | None -> vehicle.CurrentMotionParams.Acceleration

                    let newMotionParams =
                        { vehicle.CurrentMotionParams with
                              Acceleration = newAccel }

                    { vehicle with
                          CurrentMotionParams = newMotionParams }
            | None -> vehicle

        let accelerateMaximally (timeChange: TimeInterval) (vehicle: Vehicle) =
            let newMotionParams =
                { vehicle.CurrentMotionParams with
                      Acceleration = vehicle.VehicleTypeParams.MaximumParamaters.Acceleration }

            { vehicle with
                  CurrentMotionParams = newMotionParams }

        //let accelerationUpdater viewDistnace
        //                        targetSpeed
        //                        (connectionLenghtProvider: ConnectionLenghtProvider)
        //                        (timeChange: TimeInterval)
        //                        (vehicle: Vehicle)
        //                        =
        //    let len =
        //        connectionLenghtProvider vehicle.Location.Placing
        //    let travelled =
        //        Fraction.toDistance len vehicle.Location.CurrentProgress
        //    //  let targetSpeed = 1.0<m/s>
        //    let newAccel =
        //        if len - travelled < viewDistnace then
        //            if vehicle.CurrentMotionParams.Speed > targetSpeed
        //            then vehicle.VehicleTypeParams.MinimumParameters.Acceleration
        //            else 0.0<m/(s*s)>
        //        else
        //            vehicle.VehicleTypeParams.MaximumParamaters.Acceleration
        //    let newMotionParams =
        //        { vehicle.CurrentMotionParams with
        //              Acceleration = newAccel }
        //    { vehicle with
        //          CurrentMotionParams = newMotionParams }

        let speedUpdater (timeChange: TimeInterval) vehicle =
            let (TimeInterval dt) = timeChange

            let newSpeed =
                vehicle.CurrentMotionParams.Speed
                + (vehicle.CurrentMotionParams.Acceleration * dt)

            let maxSpeed =
                vehicle.VehicleTypeParams.MaximumParamaters.Speed

            let minSpeed =
                vehicle.VehicleTypeParams.MinimumParameters.Speed

            let speedCapped =
                if newSpeed > minSpeed
                then (if newSpeed < maxSpeed then newSpeed else maxSpeed)
                else minSpeed

            let newMotionParams =
                { vehicle.CurrentMotionParams with
                      Speed = speedCapped }

            { vehicle with
                  CurrentMotionParams = newMotionParams }

        let locationUpdater (obstacleFinder: Obstacles.ObstacleFinder)
                            (conectionLenghtProvider: ConnectionLenghtProvider)
                            (nextConnectionChooser: NextConnectionChooser)
                            (timeChange: TimeInterval)
                            (vehicle: Vehicle)
                            =
            let distanceTravelled =
                calculateDistanceTravelled vehicle.CurrentMotionParams.Speed timeChange

            if distanceTravelled = 0.0<m> then
                (vehicle, None)
            else
                let rec locationUpdaterHelper conectionLenghtProvider
                                              (nextConnectionChooser: NextConnectionChooser)
                                              currentConnection
                                              distanceTravelled
                                              initialDistance
                                              =
                    let lenght =
                        conectionLenghtProvider currentConnection

                    let buildResult newDistance (collision: Collision option) =
                        { ObjectLocation.CurrentProgress = Fraction.fromDistance newDistance lenght
                          ObjectLocation.Placing = currentConnection },
                        collision

                    let buildResultNoCollision newDistance = buildResult newDistance None

                    optional {
                        let! (obstacle, distance) = obstacleFinder vehicle

                        if (distance < distanceTravelled) then
                            let collision =
                                optional {
                                    let! v2 = obstacle.toVehicle ()
                                    return Collisions.fromVehicles vehicle v2
                                }

                            return buildResult (initialDistance + distance) collision

                    }
                    |> Option.defaultWith (fun () ->
                        if initialDistance + distanceTravelled <= lenght then
                            buildResultNoCollision (initialDistance + distanceTravelled)
                        else
                            let nextConnection =
                                nextConnectionChooser currentConnection.EndId

                            match nextConnection with
                            | None -> buildResultNoCollision lenght
                            | Some (nextConnection) ->
                                let locationUpdaterHelper =
                                    locationUpdaterHelper
                                        conectionLenghtProvider
                                        (nextConnectionChooser: NextConnectionChooser)

                                let distanceLeft =
                                    (distanceTravelled - (lenght - initialDistance))

                                locationUpdaterHelper nextConnection distanceLeft 0.0<m>)

                let currentConnection = vehicle.Location.Placing

                let distanceFromStart =
                    vehicle.Location.CurrentProgress
                    |> Fraction.toDistance (conectionLenghtProvider currentConnection)

                let (newLocation, collision) =
                    locationUpdaterHelper
                        conectionLenghtProvider
                        nextConnectionChooser
                        currentConnection
                        distanceTravelled
                        distanceFromStart

                { vehicle with Location = newLocation }, collision

    type VehicleWithComputation<'TResult> =
        { Vehicle: Vehicle
          Computation: Stateful.Stateful<Vehicle, 'TResult> }

    type SequenceComputation<'TResult> = VehicleWithComputation<'TResult> seq -> (Vehicle * 'TResult) seq

    module SequenceComputation =

        let update (computations: VehicleWithComputation<'TResult> seq) =
            computations
            |> PSeq.map (fun w -> (Stateful.execute w.Vehicle w.Computation)) |> PSeq.toArray |> Seq.ofArray
        //|> Seq.map (fun (v, _) -> v)
        let updateByPlacing (sequenceUpdater: SequenceComputation<'TResult>)
                            (computations: VehicleWithComputation<'TResult> seq)
                            =
            computations
            |> PSeq.groupBy (fun w -> w.Vehicle.Location.Placing)
            |> PSeq.map
                ((fun (_, vehicles) ->
                    vehicles
                    |> Seq.sortByDescending (fun w -> w.Vehicle.Location.CurrentProgress))
                 >> (fun vehiclesOnSameConnection -> sequenceUpdater vehiclesOnSameConnection))
            |> PSeq.fold (fun acc cur -> acc |> Seq.append cur) Seq.empty
