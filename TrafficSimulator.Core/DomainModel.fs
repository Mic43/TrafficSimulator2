namespace TrafficSimulator.Core

open Microsoft.FSharp.Data.UnitSystems.SI.UnitSymbols
open System
open Common
open BaseTypes
open QuadraticBezier

module DomainModel =

    type Crossing =
        { Name: option<string>
          Position: Position }

    type ConnectionType =
        | Linear
        | QuadraticBezier of QuadraticBezier

    type Connection =
        { ConnectionType: ConnectionType
          StartId: CrossingId
          EndId: CrossingId }

    type ObjectLocation =
        { Placing: Connection
          CurrentProgress: Fraction }


    type VehicleMotionParams =
        { Acceleration: float<m / (s * s)>
          Speed: float<m / s> }

    type VehicleTypeParams =
        { MaximumParamaters: VehicleMotionParams
          MinimumParameters: VehicleMotionParams }

    module VehicleTypes =
        let type1 =
            { MaximumParamaters =
                  { Acceleration = 6.0<m/(s*s)>
                    Speed = 10.0<m/s> }
              MinimumParameters =
                  { Acceleration = -20.0<m/(s*s)>
                    Speed = 0.0<m/s> } }

        let type2 =
            { MaximumParamaters =
                  { Acceleration = 8.0<m/(s*s)>
                    Speed = 13.0<m/s> }
              MinimumParameters =
                  { Acceleration = -20.0<m/(s*s)>
                    Speed = 0.0<m/s> } }

    type GraphPath =
        private { ByCrossing: Map<CrossingId, Connection>
                  ConnectionsSequence: Connection seq }

    type CollidedState = { StateDuration: TimeInterval }

    type VehicleState =
        | Running
        | Collided of CollidedState
        | ToBeRemoved

    [<CustomEquality; CustomComparison>]
    type Vehicle =

        { Id: VehicleId
          Location: ObjectLocation
          CurrentMotionParams: VehicleMotionParams
          VehicleTypeParams: VehicleTypeParams
          DrivePath: GraphPath option
          State: VehicleState }
        member this.isBeforeOnSameConnection other = 
            (this.Location.Placing = other.Location.Placing) && this.Location.CurrentProgress <= other.Location.CurrentProgress
        member this.makeStopped() =
            { this with
                  CurrentMotionParams =
                      { this.CurrentMotionParams with
                            Speed = 0.0<m/s> } }

        member this.makeImmovable() =
            let stopped = this.makeStopped ()

            { stopped with
                  VehicleTypeParams =
                      { stopped.VehicleTypeParams with
                            MaximumParamaters =
                                { stopped.VehicleTypeParams.MaximumParamaters with
                                      Speed = 0.0<m/s> } } }

        override this.Equals(other) =
            match other with
            | :? Vehicle as other -> this.Id = other.Id
            | _ -> Object.Equals(this, other)

        override this.GetHashCode() = hash this.Id

        interface System.IComparable with
            member this.CompareTo other =
                match other with
                | :? Vehicle as other -> Operators.compare this.Id other.Id
                | _ -> 1

    type ConnectionsGraph =
        private { Crossings: Map<CrossingId, Crossing>
                  CrossingsOutput: Map<CrossingId, Connection seq> }

    module ConnectionsGraph =
        let create (crossings: Map<CrossingId, Crossing>) (connections: Connection seq) =
            if not
                (connections
                 |> Seq.forall (fun connection ->
                     crossings.ContainsKey connection.StartId
                     && crossings.ContainsKey connection.EndId)) then
                None
            else
                let crossingOut =
                    connections
                    |> Seq.groupBy (fun con -> con.StartId)
                    |> Map.ofSeq

                Some
                    ({ Crossings = crossings
                       CrossingsOutput = crossingOut })

        let crossings connectionsGraph = connectionsGraph.Crossings

        let crossing connectionsGraph crossingId =
            connectionsGraph.Crossings |> Map.find crossingId

        let crossingOutputs connectionsGraph crossingId =
            if not
                (connectionsGraph.Crossings
                 |> Map.containsKey crossingId) then
                invalidArg "crossingId" "Given crossing was not found"
            else
                Map.tryFind crossingId connectionsGraph.CrossingsOutput
                |> Option.defaultValue Seq.empty

        let containsConnection connectionsGraph connection =
            connectionsGraph.Crossings
            |> Map.containsKey connection.EndId
            && connectionsGraph.Crossings
               |> Map.containsKey connection.StartId

        let calculateCrossingInputs connectionsGraph crossingId =
            connectionsGraph.CrossingsOutput
            |> Map.toSeq
            |> Seq.map (fun (_, coll) -> coll)
            |> Seq.collect id
            |> Seq.filter (fun con -> con.StartId = crossingId)

    module GraphPath =
        let tryCreate (connectionsGraph: ConnectionsGraph) (connectionsToVisit: Connection seq) =
            if not
                (connectionsToVisit
                 |> Seq.forall (fun con ->
                     con
                     |> ConnectionsGraph.containsConnection connectionsGraph)) then
                None
            else if not
                        (connectionsToVisit
                         |> Seq.pairwise
                         |> Seq.forall (fun (conn1, conn2) -> conn1.EndId = conn2.StartId)) then
                None
            else
                Some
                    { ByCrossing =
                          (connectionsToVisit
                           |> Seq.map (fun conn -> conn.StartId, conn)
                           |> Map.ofSeq)
                      ConnectionsSequence = connectionsToVisit }
        let create (connectionsGraph: ConnectionsGraph) (connectionsToVisit: Connection seq) = 
            tryCreate connectionsGraph connectionsToVisit |> Option.defaultWith (fun () -> failwith "error creating path")
        let fromSingleConnection connection =
            { ByCrossing = Map.empty.Add(connection.StartId, connection)
              ConnectionsSequence = [ connection ] }

        let tryGetConnection crossingId (driveSchedule: GraphPath) =
            driveSchedule.ByCrossing |> Map.tryFind crossingId

        let getConnectionsSequence (driveSchedule: GraphPath) = driveSchedule.ConnectionsSequence

    type Connection with
        member this.Start(cg: ConnectionsGraph) =
            ConnectionsGraph.crossing cg this.StartId

        member this.End(cg: ConnectionsGraph) = ConnectionsGraph.crossing cg this.EndId

    type VehicleDistanceAccelerationMap = Map<Distance, Acceleration>

module DomainFunctions =
    open DomainModel

    let bezierFromCrossings crossingsMap crossingStartId crossingEndId controlPolint =
        let crossing1 = crossingsMap |> Map.find crossingStartId
        let crossing2 = crossingsMap |> Map.find crossingEndId

        { ConnectionType =
              ConnectionType.QuadraticBezier
                  (QuadraticBezier.create crossing1.Position controlPolint crossing2.Position 500)
          StartId = crossingStartId
          EndId = crossingEndId }

    type ConnectionLenghtProvider = Connection -> Distance
    type NextConnectionChooser = CrossingId -> Connection option

    module LenghtProviders =
        let lenghtProvider (distancePerUnit: float<m>) (connectionsGraph: ConnectionsGraph) connection =
            match connection.ConnectionType with
            | QuadraticBezier bezier ->
                let p0 =
                    (connection.Start connectionsGraph).Position

                let p1 = bezier.controlPoint

                let p2 =
                    (connection.End connectionsGraph).Position

                (QuadraticBezier.calculateLenght p0 p1 p2)
                * distancePerUnit
            | Linear ->
                let endC = connection.End connectionsGraph
                let startC = connection.Start connectionsGraph

                let distance =
                    Position.distance endC.Position startC.Position

                distance * distancePerUnit

    let calculateDistanceTravelled (speed: Speed) (TimeInterval timeChange) = speed * timeChange

    let locationToPosition (connectionsGraph: ConnectionsGraph) (position: ObjectLocation) =
        let startPos =
            (position.Placing.Start connectionsGraph).Position

        let endPos =
            (position.Placing.End connectionsGraph).Position


        match (startPos, endPos) with
        | (Position2d pos1, Position2d pos2) ->
            match position.Placing.ConnectionType with
            | Linear ->
                let currentPos = position.CurrentProgress.Value

                { X = pos1.X + (pos2.X - pos1.X) * currentPos
                  Y = pos1.Y + (pos2.Y - pos1.Y) * currentPos }
            | QuadraticBezier b ->
                let n = (b.segmentsCache.Length - 1) |> float

                let temp =
                    ((position.CurrentProgress.Value * n) |> int)

                let t = b.segmentsCache.[temp]

                let t2 =
                    if temp + 1 < b.segmentsCache.Length then b.segmentsCache.[temp + 1] else t
                //let t = Map.find (b.segmentsCache |> Map.findKey (fun fraction _ -> position.CurrentProgress <= fraction)) b.segmentsCache
                let point1 = b.calculate t pos1 pos2
                let point2 = b.calculate t2 pos1 pos2
                Position.lerp point1 point2 0.5

    let getVehiclesOnConnection connection vehicles =
        vehicles
        |> Seq.filter (fun v -> v.Location.Placing = connection)

    module NextConnectionChoosers =
        let chooseFirst connectionsGraph crossingId =
            ConnectionsGraph.crossingOutputs connectionsGraph crossingId
            |> Seq.tryHead
        let chooseLast connectionsGraph crossingId =
            ConnectionsGraph.crossingOutputs connectionsGraph crossingId
            |> Seq.tryLast

        let chooseRandom connectionsGraph crossingId =
            let outputs =
                crossingId
                |> ConnectionsGraph.crossingOutputs connectionsGraph

            if outputs |> Seq.isEmpty then
                None
            else
                outputs
                |> Seq.item ((Seq.length outputs) |> Random().Next)
                |> Some
        let chooseRandom2 (vehicle:Vehicle) connectionsGraph crossingId =
            let outputs =
                crossingId
                |> ConnectionsGraph.crossingOutputs connectionsGraph

            if outputs |> Seq.isEmpty then
                None
            else
                let (VehicleId id) = vehicle.Id
                let (CrossingId crossingId) = crossingId
                outputs
                |> Seq.item ((id + crossingId) % (Seq.length outputs))
                |> Some
        let chooseByDrivePath schedule crossingId =
            schedule |> GraphPath.tryGetConnection crossingId

        let doNotTurnBack (vehicle: Vehicle) (chooser: NextConnectionChooser) connectionsGraph crossingId =
            let perform = chooser crossingId

            let rec repeat () =
                match perform with
                | Some c ->
                    if c.StartId = vehicle.Location.Placing.EndId
                       && (ConnectionsGraph.crossingOutputs connectionsGraph crossingId)
                          |> Seq.length = 1 then
                        repeat ()
                    else
                        Some(c)
                | None -> None

            repeat ()
