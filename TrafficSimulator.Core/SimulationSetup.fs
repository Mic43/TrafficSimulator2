namespace TrafficSimulator.Core

open Microsoft.FSharp.Data.UnitSystems.SI.UnitSymbols
open Common
open BaseTypes
open DomainModel
open DomainFunctions
open Vehicles
open Collisions

type SimulationState =
    { ConnectionsGraph: ConnectionsGraph
      Vehicles: Vehicle Set
      Collisions: Collision Set
      TrafficLights: TrafficLights.LightSystem Set } // TODO: change to map by id

module Setup =

    let private distancePerUnit = 10.0<m>
    let private collisionDuration = TimeInterval.create 5.0<s>


    let private nextConnectionChooserFactory (vehicle: Vehicle) connectionsGraph =
        match vehicle.DrivePath with
        | Some (schedule) -> NextConnectionChoosers.chooseByDrivePath schedule
        | None -> NextConnectionChoosers.chooseRandom connectionsGraph

    let private connectionLenghtProvider connectionsGraph =
        LenghtProviders.lenghtProvider distancePerUnit connectionsGraph

    let private buildAccelerationUpdater connectionLenghtProvider simulationState =
        let vehiclleAccelMapHard = Map.empty
                                    .Add( 3.0<m>,-10.0<m/(s*s)>)
                                    .Add( 2.0<m>,-20.0<m/(s*s)>)
                                    .Add( 1.0<m>,-75.0<m/(s*s)>)
                                    //.Add( 0.5<m>,-150.0<m/(s*s)>)
        let vehiclleAccelMapLight =
            Map.empty.Add(4.0<m>, -2.0<m/(s*s)>)
                     .Add(3.0<m>, -8.0<m/(s*s)>)

        let vehiclesFinder =
            Obstacles.Finders.nearestVehiceAheadOnSameConnection connectionLenghtProvider simulationState.Vehicles

        let crossingFinder =
            Obstacles.Finders.nextCrossing simulationState.ConnectionsGraph connectionLenghtProvider

        let trafficLightsFinder =
            Obstacles.Finders.nearestRedLightAheadOnSameConnection connectionLenghtProvider simulationState.TrafficLights

        let nearestFinder =
            Obstacles.Finders.nearestObstacle
                (seq {
                    vehiclesFinder
                    trafficLightsFinder
                 })

        //let finder = ObstacleFinders.nearestObstacle finders
        let updaters =
            seq {
                Updaters.accelerateMaximally

                Updaters.breakOnlyIfSpeedGreaterThan
                    5.0<m/s>
                    (Updaters.breakByDistance vehiclleAccelMapLight crossingFinder)

                Updaters.breakOnlyIfSpeedGreaterThan
                    0.0<m/s>
                    (Updaters.breakByDistance vehiclleAccelMapHard nearestFinder)
            }

        Updaters.compositeUpdater updaters

    module Computation =
        open Common.Stateful
        open SequenceComputation

        let buildComputationForVehicle simulationState timeChange (vehicle: Vehicle) =
            let connectionsGraph = simulationState.ConnectionsGraph

            let connectionLenghtProvider =
                connectionLenghtProvider connectionsGraph

            let nextConnectionChooser =
                (nextConnectionChooserFactory vehicle connectionsGraph)

            let collisionsVehiclesFinder =
                Obstacles.Finders.nearestVehiceAheadOnSameConnection connectionLenghtProvider simulationState.Vehicles

            let vehicleStateUpdater = 
                fromAction (Updaters.stateUpdater collisionDuration simulationState.Collisions timeChange)
            let accelerationUpdater =
                fromAction (buildAccelerationUpdater connectionLenghtProvider simulationState timeChange)
            let speedUpdater =
                fromAction (Updaters.speedUpdater timeChange)
            let locationUpdater =
                Stateful
                    (Updaters.locationUpdater
                        collisionsVehiclesFinder
                         connectionLenghtProvider
                         nextConnectionChooser
                         timeChange)

            stateful {
                do! vehicleStateUpdater
                do! accelerationUpdater
                do! speedUpdater
                return! locationUpdater
            }

        let buildVehiclesSequenceComputation simulationState =
            let ret timeChange =
                let computations =
                    simulationState.Vehicles
                    |> Seq.map (fun vehicle ->
                        { Vehicle = vehicle
                          Computation = (buildComputationForVehicle simulationState timeChange vehicle) })

                let innerVehiclesUpdater = SequenceComputation.update

                let decoratedVehiclesUpdater =
                    SequenceComputation.updateByPlacing innerVehiclesUpdater computations

                decoratedVehiclesUpdater

            ret

    module ApiFunctions =
        open TrafficLights

        let updateSimulationState simulationState timeInterval =      

            let vehicleSequenceComputation =
                Computation.buildVehiclesSequenceComputation simulationState

            let (newVehicles, newCollisions) =
                vehicleSequenceComputation timeInterval
                |> Seq.toList
                |> List.unzip

            let newCollisions =
                newCollisions |> List.choose id |> Set.ofList

            let newVehicles = 
                newVehicles|> Set.ofList |> Set.filter (fun v -> v.State = VehicleState.ToBeRemoved |> not)

            let newTrafficLights =
                simulationState.TrafficLights
                |> Set.map (fun tl -> tl |> Functions.updateLightSystem timeInterval)

            { simulationState with
                  Vehicles = newVehicles 
                  TrafficLights = newTrafficLights 
                  Collisions = newCollisions }

        let init () =
            let crossings =
                Map.empty.Add(CrossingId 1,
                              { Name = Some "aaa"
                                Position = Position2d { X = 1.0; Y = 2.0 } })
                   .Add(CrossingId 2,
                        { Name = None
                          Position = Position2d { X = 1.5; Y = 4.0 } })
                   .Add(CrossingId 3,
                        { Name = None
                          Position = Position2d { X = 5.5; Y = 4.0 } })
                   .Add(CrossingId 4,
                        { Name = None
                          Position = Position2d { X = 3.5; Y = 0.5 } })
                   .Add(CrossingId 5,
                        { Name = None
                          Position = Position2d { X = 0.5; Y = 1.5 } })
                   .Add(CrossingId 6,
                        { Name = None
                          Position = Position2d { X = 3.0; Y = 0.0 } })

            let connections =
                [ DomainFunctions.bezierFromCrossings
                    crossings
                      (CrossingId 1)
                      (CrossingId 2)
                      (Position2d { X = 0.5; Y = 2.5 })
                  DomainFunctions.bezierFromCrossings
                      crossings
                      (CrossingId 2)
                      (CrossingId 1)
                      (Position2d { X = 0.5; Y = 2.5 })
                  { ConnectionType = Linear
                    StartId = CrossingId 1
                    EndId = CrossingId 3 }
                  { ConnectionType = Linear
                    StartId = CrossingId 3
                    EndId = CrossingId 1 }
                  DomainFunctions.bezierFromCrossings
                      crossings
                      (CrossingId 1)
                      (CrossingId 4)
                      (Position2d { X = 1.5; Y = 1.0 })
                  DomainFunctions.bezierFromCrossings
                      crossings
                      (CrossingId 4)
                      (CrossingId 1)
                      (Position2d { X = 1.5; Y = 1.0 })
                  { ConnectionType = Linear
                    StartId = CrossingId 1
                    EndId = CrossingId 5 }
                  { ConnectionType = Linear
                    StartId = CrossingId 5
                    EndId = CrossingId 1 }
                  DomainFunctions.bezierFromCrossings
                      crossings
                      (CrossingId 5)
                      (CrossingId 6)
                      (Position2d { X = 1.5; Y = 0.0 })
                  DomainFunctions.bezierFromCrossings
                      crossings
                      (CrossingId 6)
                      (CrossingId 5)
                      (Position2d { X = 1.5; Y = 0.0 }) ]

            let connectionsGraph =
                ConnectionsGraph.create crossings connections

            let createVehicle initialSpeed id vehicleType start =
                { Id = VehicleId id
                  CurrentMotionParams =
                      { Speed = initialSpeed
                        Acceleration = 0.0<m/(s*s)> }
                  VehicleTypeParams = vehicleType
                  Vehicle.Location =
                      { ObjectLocation.CurrentProgress = (Fraction.tryCreate start) |> Option.get
                        Placing = connections.[1] }
                  DrivePath = None 
                  State = VehicleState.Running
                  }

            let count = 20

            let vehicles =
                Seq.init count (fun i ->
                    createVehicle
                        0.0<m/s>
                        i
                        (if (i % 2 = 0) then VehicleTypes.type1 else VehicleTypes.type2)
                        ((float i) / (float count)))
                |> Set.ofSeq

            let greenRedInterval = (TimeInterval.create 5.0<s>)
            let greenRedInterval2 = (TimeInterval.create 3.0<s>)

            let trafficLights =
                [ Functions.createCrossingLightSystem
                    (set [ connections.[1]; connections.[5] ])
                      (set [ connections.[3]; connections.[7] ])
                      greenRedInterval
                      greenRedInterval
                      (connectionLenghtProvider connectionsGraph.Value)
                  Functions.createSimpleLightSystem
                      greenRedInterval
                      greenRedInterval2
                      connections.[2]
                      connections.[3]
                      (Fraction.create 0.5) ]

            (optional {
                let! cG = connectionsGraph
                let tl = trafficLights |> List.choose id

                return
                    { SimulationState.Vehicles = vehicles
                      SimulationState.ConnectionsGraph = cG
                      TrafficLights = tl |> set 
                      Collisions = Set.empty }
             })
            |> Option.defaultWith (fun () -> failwith "Error creating graph")
