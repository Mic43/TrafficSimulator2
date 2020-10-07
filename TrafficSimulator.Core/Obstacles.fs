namespace TrafficSimulator.Core

open Common
open BaseTypes
open Microsoft.FSharp.Data.UnitSystems.SI.UnitSymbols
open DomainModel
open DomainFunctions

module Obstacles =

    type Obstacle =
        | OtherVehicle of Vehicle
        | Crossing of Crossing
        | RedLight of TrafficLights.TrafficLight

        member this.toVehicle() =
            match this with
            | OtherVehicle v -> Some(v)
            | _ -> None

    type ObstacleFinder = Vehicle -> (Obstacle * Distance) option

    module Finders =
        open TrafficLights

        let nearestVehicleAhead (path: GraphPath)
                                (connectionLenghtProvider: ConnectionLenghtProvider)
                                allVehicles
                                (vehicle: Vehicle)
                                =

            let vehicleProgress (vehicle: Vehicle) =
                (Fraction.toDistance
                    (connectionLenghtProvider vehicle.Location.Placing)
                     vehicle.Location.CurrentProgress)

            //let res = optional {
            let a =
                GraphPath.getConnectionsSequence path
                |> Seq.collect (fun c ->
                    getVehiclesOnConnection c allVehicles
                    |> Seq.sortBy (fun v -> v.Location.CurrentProgress))
                |> Seq.toList

            let nearest =
                a
                |> Seq.skipWhile (fun v ->
                    v.Id = vehicle.Id
                    || v.isBeforeOnSameConnection vehicle)
                |> Seq.toList
                |> List.tryHead

            let connection = vehicle.Location.Placing

            nearest
            |> Option.bind (fun nearest ->

                let distance =
                    GraphPath.getConnectionsSequence path
                    |> Seq.skipWhile (fun con -> con = connection |> not)
                    |> Seq.takeWhile (fun con -> con = nearest.Location.Placing |> not)
                    |> Seq.fold (fun distance con -> distance + (connectionLenghtProvider con)) 0.0<m>

                //return
                Some
                    (Obstacle.OtherVehicle nearest,
                     distance
                     + (vehicleProgress nearest)
                     - (vehicleProgress vehicle)))
        //}
        // res

        let nearestVehiceAheadOnPath connectionGraph
                                     (nextConnectionChooser: NextConnectionChooser)
                                     (connectionLenghtProvider: ConnectionLenghtProvider)
                                     allVehicles
                                     (vehicle: Vehicle)
                                     =
            let connections =
                [ Some(vehicle.Location.Placing)
                  (nextConnectionChooser vehicle.Location.Placing.EndId) ]
                |> List.choose id

            let path =
                (GraphPath.create connectionGraph connections)

            nearestVehicleAhead path connectionLenghtProvider allVehicles vehicle

        let nearestVehiceAheadOnSameConnection (connectionLenghtProvider: ConnectionLenghtProvider)
                                               allVehicles
                                               (vehicle: Vehicle)
                                               =
            nearestVehicleAhead
                (GraphPath.fromSingleConnection vehicle.Location.Placing)
                connectionLenghtProvider
                allVehicles
                vehicle
        //let currentProgress (vehicle: Vehicle) = vehicle.Location.CurrentProgress

        //let connLen =
        //    connectionLenghtProvider vehicle.Location.Placing

        //let vehicles =
        //    allVehicles
        //    |> getVehiclesOnConnection vehicle.Location.Placing
        //    |> Seq.filter (fun v -> not (v.Id = vehicle.Id))
        //    |> Seq.filter (fun v -> v.Location.CurrentProgress > vehicle.Location.CurrentProgress)
        //    |> Seq.map (fun v ->
        //        (v, Fraction.distanceOnSameConnecton (currentProgress vehicle) (currentProgress v) connLen))

        //if vehicles |> Seq.isEmpty then
        //    None
        //else
        //    match vehicles
        //          |> Seq.minBy (fun (v, distance) -> distance) with
        //    | (vehicle, distance) -> Some(Obstacle.OtherVehicle vehicle, distance)



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
