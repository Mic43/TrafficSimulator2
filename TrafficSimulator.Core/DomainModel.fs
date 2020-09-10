namespace TrafficSimulator.Core

open Microsoft.FSharp.Data.UnitSystems.SI.UnitSymbols     
open System

module DomainModel =    

    type CrossingId = CrossingId of int   
    type VehicleId = VehicleId of int
    type TimeInterval =  TimeInterval of float<s>  

    type [<Struct>] Position2d =  {X:float;Y:float}
    type Position = Position2d of Position2d 
    type Distance = float<m>
    type Speed = float<m/s>
    type Acceleration = float<m/(s*s)>

    type Curve = float    
    type ConnectionType = Linear | Curved of Curve | QuadraticBezier of Position

    module Position = 
        let distance (point:Position) (point2:Position) =
            let p = (point,point2)  
            match p with
                | (Position2d p1,Position2d p2) -> sqrt ((p1.X - p2.X) * (p1.X - p2.X) + (p1.Y - p2.Y) * (p1.Y - p2.Y)) 
                
    type Crossing = { Name:option<string> ; Position:Position} 
   
    type Fraction =  private Fraction of float with
        member this.Value = 
                             let getValue (Fraction f) = f
                             getValue this
    module Fraction =
        let create (value:float) = 
            if ((Math.Abs value) <= 1.0) 
                then Some(Fraction value)
                else None        
        let zero = Fraction 0.0
        let one = Fraction 1.0

        let fromDistance (distance:Distance) (connectionLenght:float<m>) = 
            create (distance / connectionLenght)
        let toDistance (connectionLenght:Distance) (Fraction value) =
            value*connectionLenght
        let distanceOnSameConnecton (progressFrom:Fraction) (progressTo:Fraction) connectionLenght= 
                  let dist =  create (progressTo.Value - progressFrom.Value) 
                                  |> Option.defaultWith (fun () -> failwith "Not possible to create fraction not between -1 and 1")
                  toDistance connectionLenght dist
    type Connection = {ConnectionType:ConnectionType;StartId:CrossingId;EndId:CrossingId}
    type DrivePath = private DrivePath of Map<CrossingId,Connection>
   
    type VehicleLocation = {Placing:Connection;CurrentProgress:Fraction}  
    type VehicleMotionParams = {Acceleration:float<m/(s*s)>;Speed:float<m/s>}
    type VehicleTypeParams = {MaximumParamaters:VehicleMotionParams;MinimumParameters:VehicleMotionParams}
    module VehicleTypes = 
        let type1 = {MaximumParamaters = {Acceleration = 6.0<m/(s*s)>;Speed = 10.0<m/s>};
                     MinimumParameters = {Acceleration = -20.0<m/(s*s)>;Speed = 0.0<m/s>}}
        let type2 = {MaximumParamaters = {Acceleration = 8.0<m/(s*s)>;Speed = 13.0<m/s>};
                            MinimumParameters = {Acceleration = -20.0<m/(s*s)>;Speed = 0.0<m/s>}}
    type Vehicle = {Id:VehicleId;Location:VehicleLocation;CurrentMotionParams:VehicleMotionParams;
                    VehicleTypeParams:VehicleTypeParams;DrivePath:DrivePath option}

    type ConnectionsGraph = private { Crossings:Map<CrossingId,Crossing>; CrossingsOutput: Map<CrossingId,Connection seq>;} 
    module ConnectionsGraph =         
        let create (crossings: Map<CrossingId,Crossing>) (connections: Connection seq)  = 
            if not (connections |> Seq.forall (fun connection -> crossings.ContainsKey connection.StartId && crossings.ContainsKey connection.EndId))
                then None
                else
                    let crossingOut = connections |> Seq.groupBy (fun con -> con.StartId ) |> Map.ofSeq
                    Some({Crossings = crossings;CrossingsOutput = crossingOut})
        let crossings connectionsGraph = connectionsGraph.Crossings
        let crossing connectionsGraph crossingId = connectionsGraph.Crossings  |> Map.find  crossingId    
        let crossingOutputs connectionsGraph crossingId =  
            if not (connectionsGraph.Crossings |> Map.containsKey crossingId)
                then invalidArg "crossingId" "Given crossing was not found"
            else
                Map.tryFind crossingId connectionsGraph.CrossingsOutput |> Option.defaultValue Seq.empty 
        let containsConnection connectionsGraph connection= 
            connectionsGraph.Crossings |> Map.containsKey connection.EndId && connectionsGraph.Crossings |> Map.containsKey connection.StartId 
    module DrivePath = 
        let create (connectionsGraph:ConnectionsGraph) (connectionsToVisit: Connection seq) =
            if not (connectionsToVisit |> Seq.forall (fun con -> con |> ConnectionsGraph.containsConnection connectionsGraph))
                then None
            else
                if not (connectionsToVisit |> Seq.pairwise |> Seq.forall ( fun (conn1,conn2) -> conn1.EndId = conn2.StartId)) 
                    then None
                else
                    Some(DrivePath (connectionsToVisit |> Seq.map (fun conn -> conn.StartId, conn) |> Map.ofSeq ))
        let tryGetConnection crossingId (DrivePath driveSchedule) = 
            driveSchedule |> Map.tryFind crossingId
    type SimulationState = {ConnectionsGraph:ConnectionsGraph; Vehicles: Vehicle seq} // TODO: change to map by id
    type Connection with 
            member this.Start (cg:ConnectionsGraph) = 
                ConnectionsGraph.crossing cg this.StartId
            member this.End (cg:ConnectionsGraph) = 
                ConnectionsGraph.crossing cg this.EndId   
    type VehicleDistanceAccelerationMap = Map<Distance,Acceleration> 
    type Obstacle = OtherVehicle of Vehicle | Crossing of Crossing | Tmp

 module DomainFunctions = 
    open DomainModel  
    // let distanceToProgress (distance:float<m>) (connectionLenght:float<m>) = 
    //     Progress.create (distance / connectionLenght)
    
    let bezierLen p0 p1 p2 =      
        match (p0,p1,p2) with 
            | (Position2d p0,Position2d p1,Position2d p2) ->
                let a = {X = p0.X - 2.0 * p1.X + p2.X; Y = p0.Y - 2.0 *p1.Y + p2.Y} //a.x = p0->x - 2*p1->x + p2->x; 
                let b = {X = 2.0*p1.X - 2.0*p0.X;Y = 2.0*p1.Y - 2.0*p0.Y}
       
                let A = 4.0 * (a.X*a.X + a.Y*a.Y);
                let B = 4.0 *(a.X*b.X + a.Y*b.Y);
                let C = b.X*b.X + b.Y*b.Y;
    
                let Sabc = 2.0 *sqrt(A+B+C);
                let A_2 = sqrt(A);
                let A_32 = 2.0 * A*A_2;
                let C_2 = 2.0 * sqrt(C);
                let BA = B/A_2;
    
                (A_32*Sabc + A_2*B*(Sabc-C_2) + (4.0*C*A-B*B)*log( (2.0*A_2+BA+Sabc)/(BA+C_2))) / (4.0*A_32);                
    
    type ConnectionLenghtProvider = Connection -> Distance  

    type NextConnectionChooser = CrossingId -> Connection option

    module LenghtProviders = 
        let constantLenghtProvider (connectionsGraph:ConnectionsGraph) connection  =
            match connection.ConnectionType with 
                |  Curved c-> 100.0<m>
                |  Linear -> 100.0<m>
        let lenghtProvider (distancePerUnit:float<m>) (connectionsGraph:ConnectionsGraph) connection  =
            match connection.ConnectionType with 
                |  Curved c-> failwith "Not implemented"
                |  QuadraticBezier controlPoint -> let p0 = (connection.Start connectionsGraph).Position 
                                                   let p1 = controlPoint
                                                   let p2 = (connection.End connectionsGraph).Position
                                                   (bezierLen p0 p1 p2) * distancePerUnit
                |  Linear -> let endC = connection.End connectionsGraph
                             let startC = connection.Start connectionsGraph                         
                             let distance = Position.distance endC.Position startC.Position 
                             distance * distancePerUnit
    let calculateDistanceTravelled (speed:Speed) (TimeInterval timeChange) = 
        speed * timeChange
    let calculateVehiclePosition  (connectionsGraph:ConnectionsGraph) (vehicle:Vehicle) = 
            let startPos = (vehicle.Location.Placing.Start connectionsGraph).Position
            let endPos =   (vehicle.Location.Placing.End connectionsGraph).Position 
            let t = vehicle.Location.CurrentProgress.Value

            match (startPos, endPos) with 
            |  (Position2d pos1,Position2d pos2) -> 
                match vehicle.Location.Placing.ConnectionType with
                    | Curved c -> failwith "Not implemented"
                    | Linear -> {X = pos1.X + (pos2.X - pos1.X) * t;Y = pos1.Y + (pos2.Y - pos1.Y) * t}
                    | QuadraticBezier controlPoint -> let p0 = pos1
                                                      let p1 = controlPoint
                                                      let p2 = pos2
                                                      match p1 with 
                                                      | (Position2d p1) -> 
                                                            let a = (1.0 - t) * (1.0 - t)
                                                            {X = p1.X + a * (p0.X - p1.X) + t*t*(p2.X-p1.X);
                                                             Y = p1.Y + a * (p0.Y - p1.Y) + t*t*(p2.Y-p1.Y) }
    let getVehiclesOnConnection  connection vehicles = 
         vehicles |> Seq.filter (fun v -> v.Location.Placing = connection)
        
    module NextConnectionChoosers = 
        let chooseFirst connectionsGraph crossingId =
            ConnectionsGraph.crossingOutputs connectionsGraph crossingId |> Seq.tryHead         
        let chooseRandom connectionsGraph crossingId =     
            let outputs = crossingId |> ConnectionsGraph.crossingOutputs connectionsGraph        
            if outputs |> Seq.isEmpty 
                then None
            else 
                outputs|> Seq.item ((Seq.length outputs) |> Random().Next) |> Some  
        let chooseByDrivePath schedule crossingId = 
            schedule |> DrivePath.tryGetConnection crossingId
