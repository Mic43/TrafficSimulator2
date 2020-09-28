namespace TrafficSimulator.Core

open Microsoft.FSharp.Data.UnitSystems.SI.UnitSymbols
open System
open BaseTypes
open DomainModel
open DomainFunctions
open Common

module Vehicles =
    type VehicleUpdater = TimeInterval -> Vehicle -> Vehicle
    type VehicleSequenceUpdater = TimeInterval -> Vehicle seq -> Vehicle seq

    //module UpdateWorkflow =
    //    type T = seq<VehicleUpdater>

    //    let apply (timeInterval: TimeInterval) (vehicle: Vehicle) (workflow: T) =
    //        workflow
    //        |> Seq.fold (fun vehicle vUpdater -> vUpdater timeInterval vehicle) vehicle


    type Obstacle =
        | OtherVehicle of Vehicle
        | Crossing of Crossing
        | RedLight of TrafficLights.TrafficLight

        member this.toVehicle() =
            match this with
            | OtherVehicle v -> Some(v)
            | _ -> None

    module ObstacleFinders =
        open TrafficLights
        type ObstacleFinder = Vehicle -> (Obstacle * Distance) option

        let nearestVehiceAheadOnSameConnection (connectionLenghtProvider: ConnectionLenghtProvider)
                                               allVehicles
                                               (vehicle: Vehicle)
                                               =
            let currentProgress (vehicle: Vehicle) = vehicle.Location.CurrentProgress

            let connLen =
                connectionLenghtProvider vehicle.Location.Placing

            let vehicles =
                allVehicles
                |> getVehiclesOnConnection vehicle.Location.Placing
                |> Seq.filter (fun v -> not (v.Id = vehicle.Id))
                |> Seq.filter (fun v -> v.Location.CurrentProgress > vehicle.Location.CurrentProgress)
                |> Seq.map (fun v ->
                    (v, Fraction.distanceOnSameConnecton (currentProgress vehicle) (currentProgress v) connLen))

            if vehicles |> Seq.isEmpty then
                None
            else
                match vehicles
                      |> Seq.minBy (fun (v, distance) -> distance) with
                | (vehicle, distance) -> Some(Obstacle.OtherVehicle vehicle, distance)


        // TODO: To similiart to nearestVehicleFinding
        let nearestRedLightAheadOnSameConnection (connectionLenghtProvider: ConnectionLenghtProvider)
                                                 (trafficLights: LightSystem Set)
                                                 (vehicle: Vehicle)
                                                 =
            let connLen =
                connectionLenghtProvider vehicle.Location.Placing

            let allLights =
                trafficLights
                |> Set.map (fun tl -> tl.getAllTrafficLights ())
                |> Set.fold (fun sum set -> sum |> Set.union set) Set.empty

            let lights =
                allLights
                |> Set.filter (fun l -> l.Location.Placing = vehicle.Location.Placing)
                |> Seq.filter (fun l -> l.State = LightState.Red)
                |> Seq.filter (fun l -> l.Location.CurrentProgress > vehicle.Location.CurrentProgress)
                |> Seq.map (fun l ->
                    (l,
                     Fraction.distanceOnSameConnecton
                         (vehicle.Location.CurrentProgress)
                         (l.Location.CurrentProgress)
                         connLen))

            if lights |> Seq.isEmpty then
                None
            else
                match lights
                      |> Seq.minBy (fun (_, distance) -> distance) with
                | (light, distance) -> Some(Obstacle.RedLight light, distance)

        let nextCrossing (connectionGraph: ConnectionsGraph)
                         (connectionLenghtProvider: ConnectionLenghtProvider)
                         (vehicle: Vehicle)
                         =
            let connectionLen =
                connectionLenghtProvider vehicle.Location.Placing

            Some
                (Obstacle.Crossing
                    (vehicle.Location.Placing.EndId
                     |> ConnectionsGraph.crossing connectionGraph),
                 Fraction.distanceOnSameConnecton vehicle.Location.CurrentProgress Fraction.one connectionLen)

        let nearestObstacle (finders: ObstacleFinder seq) vehicle =
            let obstacles =
                finders
                |> Seq.map (fun finder -> finder vehicle)
                |> Seq.collect (fun obstalce_distance -> Option.toList obstalce_distance)

            if obstacles |> Seq.isEmpty then
                None
            else
                Some
                    (obstacles
                     |> Seq.minBy (fun (_, distance) -> distance))

    type Collidable = Vehicle of Vehicle

    type Collision =
        private { Collidables: Set<Collidable> }

    module Collision =
        let create first second =
            if first = second
            then invalidArg "first" "Cannot create collision of object with itself"
            else { Collidables = [ first; second ] |> Set.ofList }

        let fromVehicles (first: Vehicle) (second: Vehicle) =
            create (Collidable.Vehicle first) (Collidable.Vehicle second)

        let (|Collision|) collision =
            (collision.Collidables.MinimumElement, collision.Collidables.MaximumElement)

        let resolve allVehicles (collisions: Collision Set) =
            let flattenSet =
                collisions
                |> Set.fold (fun acc s -> acc |> (Set.union s.Collidables)) Set.empty

            let vehiclesSet =
                flattenSet
                |> Set.map (fun collidable ->
                    match collidable with
                    | Vehicle v -> v.makeImmovable())

            let res = (Set.difference allVehicles vehiclesSet) |> Set.union vehiclesSet
            res

    module Updaters =

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
                            (obstacleFinder: ObstacleFinders.ObstacleFinder)
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

        let locationUpdater (obstacleFinder: ObstacleFinders.ObstacleFinder)
                            (conectionLenghtProvider: ConnectionLenghtProvider)
                            (nextConnectionChooser: NextConnectionChooser)
                            (timeChange: TimeInterval)
                            (vehicle: Vehicle)
                            =
            let distanceTravelled =
                calculateDistanceTravelled vehicle.CurrentMotionParams.Speed timeChange

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
                                return Collision.fromVehicles vehicle v2
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

    //type VehicleWithWorflow =
    //    { Vehicle: Vehicle
    //      Workflow: UpdateWorkflow.T }

    type VehicleWithComputation<'TResult> =
        { Vehicle: Vehicle
          Computation: Stateful.Stateful<Vehicle, 'TResult> }

    type SequenceComputation<'TResult> = VehicleWithComputation<'TResult> seq -> (Vehicle * 'TResult) seq

    module SequenceComputation =

        let update (computations: VehicleWithComputation<'TResult> seq) =
            computations
            |> Seq.map (fun w -> (Stateful.execute w.Vehicle w.Computation))
        //|> Seq.map (fun (v, _) -> v)
        let updateByPlacing (sequenceUpdater: SequenceComputation<'TResult>)
                            (computations: VehicleWithComputation<'TResult> seq)
                            =
            computations
            |> Seq.groupBy (fun w -> w.Vehicle.Location.Placing)
            |> Seq.map
                ((fun (_, vehicles) ->
                    vehicles
                    |> Seq.sortByDescending (fun w -> w.Vehicle.Location.CurrentProgress))
                 >> (fun vehiclesOnSameConnection -> sequenceUpdater vehiclesOnSameConnection))
            |> Seq.fold (fun acc cur -> acc |> Seq.append cur) Seq.empty
//module SequenceUpdaters =
//    type T = VehicleWithWorflow seq -> TimeInterval -> Vehicle seq

//    let update (workflows: VehicleWithWorflow seq) (timeChange: TimeInterval) =
//        workflows
//        |> Seq.map (fun w -> UpdateWorkflow.apply timeChange w.Vehicle w.Workflow)

//    let updateByPlacing (sequenceUpdater: T) (workflows: VehicleWithWorflow seq) (timeChange: TimeInterval) =
//        workflows
//        |> Seq.groupBy (fun w -> w.Vehicle.Location.Placing)
//        |> Seq.map
//            ((fun (_, vehicles) ->
//                vehicles
//                |> Seq.sortByDescending (fun w -> w.Vehicle.Location.CurrentProgress))
//             >> (fun vehiclesOnSameConnection ->
//                 timeChange
//                 |> sequenceUpdater vehiclesOnSameConnection))
//        |> Seq.fold (fun acc cur -> acc |> Seq.append cur) Seq.empty
