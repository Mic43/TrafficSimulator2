namespace TrafficSimulator.Core

open BaseTypes
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
