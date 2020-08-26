namespace TrafficSimulator.Core

open Microsoft.FSharp.Data.UnitSystems.SI.UnitSymbols     
open System

module DomainModel =    

    type CrossingId = CrossingId of int   
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
            if (value >= 0.0 && value <= 1.0) 
                then Some(Fraction value)
                else None        
        let zero = Fraction 0.0
        let one = Fraction 1.0

        let fromDistance (distance:float<m>) (connectionLenght:float<m>) = 
            create (distance / connectionLenght)
        let toDistance (connectionLenght:float<m>) (Fraction value) =
            value*connectionLenght
  
    type Connection = {ConnectionType:ConnectionType;StartId:CrossingId;EndId:CrossingId}
    type DrivePath = private DrivePath of Map<CrossingId,Connection>
   
    type VehicleLocation = {Placing:Connection;CurrentProgress:Fraction}  
    type VehicleMotionParams = {Acceleration:float<m/(s*s)>;Speed:float<m/s>}
    type VehicleTypeParams = {MaximumParamaters:VehicleMotionParams;MinimumParameters:VehicleMotionParams}
    module VehicleTypes = 
        let type1 = {MaximumParamaters = {Acceleration = 6.0<m/(s*s)>;Speed = 10.0<m/s>};
                     MinimumParameters = {Acceleration = -10.0<m/(s*s)>;Speed = -1.0<m/s>}}
    type Vehicle = {Location:VehicleLocation;CurrentMotionParams:VehicleMotionParams;
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
    type SimulationState = {ConnectionsGraph:ConnectionsGraph; Vehicles: Vehicle seq}
    type Connection with 
            member this.Start (cg:ConnectionsGraph) = 
                ConnectionsGraph.crossing cg this.StartId
            member this.End (cg:ConnectionsGraph) = 
                ConnectionsGraph.crossing cg this.EndId   
    type VehicleDistanceAccelerationMap = Map<Distance,Acceleration> 

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
    
    type ConnectionLenghtProvider = Connection -> float<m>    
    type VehicleUpdater = TimeInterval->Vehicle->Vehicle
    type VehicleLocationUpdater = VehicleLocation->VehicleLocation
   // type ProgressTravelledCalculator = unit -> Progress
    type VehiclesUpdater = TimeInterval->Vehicle seq->Vehicle seq
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
    let calculateDistanceTravelled (speed:float<m/s>) (TimeInterval timeChange) = 
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

    module VehicleLocationUpdaters = 
        //stopping when encounters crossing
        // let simpleLocationUpdater  (progressTravelled: Progress) (vehicleLocation:VehicleLocation)= 
        //     let newProgres = Progress.add (Progress.fromFraction vehicleLocation.CurrentProgress) progressTravelled      
        //     {vehicleLocation with CurrentProgress = newProgres.Fractions}

        // // always choosing first route on crossing
        // let locationUpdater (connectionsGraph:ConnectionsGraph) (progressTravelled: Progress) (vehicleLocation:VehicleLocation) = 
        //     let newProgres = Progress.add (Progress.fromFraction vehicleLocation.CurrentProgress) progressTravelled      
        //     let destination = ConnectionsGraph.crossingOutputs connectionsGraph vehicleLocation.Placing.EndId |> Seq.tryHead
        //     let newPlacing = if newProgres.Unities < 1      
        //                         then vehicleLocation.Placing 
        //                         else destination |> Option.defaultValue vehicleLocation.Placing
        //     {CurrentProgress = newProgres.Fractions;Placing = newPlacing}
        // let locationUpdater (nextConnectionChooser:NextConnectionChooser) (progressTravelled: Progress) (vehicleLocation:VehicleLocation)  = 
        //     let newProgres = Progress.add (Progress.fromFraction vehicleLocation.CurrentProgress) progressTravelled      
        //     let newPlacing = if newProgres.Unities < 1      
        //                         then vehicleLocation.Placing 
        //                         else (nextConnectionChooser vehicleLocation.Placing.EndId |> Option.defaultValue vehicleLocation.Placing) 
        //     {CurrentProgress = newProgres.Fractions;Placing = newPlacing}
       
        let locationUpdater (conectionLenghtProvider:ConnectionLenghtProvider) (nextConnectionChooser:NextConnectionChooser) (distance:float<m>) (vehicleLocation:VehicleLocation) =             
            let rec locationUpdaterHelper conectionLenghtProvider (nextConnectionChooser:NextConnectionChooser) currentConnection distance initialDistance  =             
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
            let currentConnection = vehicleLocation.Placing
            let currentDistance = vehicleLocation.CurrentProgress |> Fraction.toDistance (conectionLenghtProvider currentConnection)          
            locationUpdaterHelper conectionLenghtProvider nextConnectionChooser currentConnection distance currentDistance

    // updates only location of a vehicle
    module VehicleUpdaters = 
        let simpleVehicleUpdater (vehicleLocationUpdater:VehicleLocationUpdater) vehicle =  
            let newVehicleLocation = vehicleLocationUpdater vehicle.Location
            {vehicle with Location = newVehicleLocation}
        let vehicleSpeedUpdater (innerVehicleUpdater:VehicleUpdater) (timeChange:TimeInterval) vehicle = 
            let (TimeInterval dt) = timeChange
            let newSpeed =  vehicle.CurrentMotionParams.Speed + (vehicle.CurrentMotionParams.Acceleration * dt) 
            let maxSpeed = vehicle.VehicleTypeParams.MaximumParamaters.Speed
            let minSpeed = vehicle.VehicleTypeParams.MinimumParameters.Speed

            let speedCapped = if newSpeed > minSpeed then (if newSpeed < maxSpeed then newSpeed else maxSpeed) else minSpeed
            let newMotionParams = {vehicle.CurrentMotionParams with Speed = speedCapped}
            innerVehicleUpdater timeChange {vehicle with CurrentMotionParams = newMotionParams}
        let vehicleAccelerationUpdater viewDistnace targetSpeed (connectionLenghtProvider:ConnectionLenghtProvider) 
                                       (innerVehicleUpdater:VehicleUpdater) (timeChange:TimeInterval) vehicle = 
            //let viewDistnace = 5.0<m>
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
            innerVehicleUpdater timeChange {vehicle with CurrentMotionParams = newMotionParams}        
        
        type ByDistanceAccelerationChooser = Distance->Acceleration
        let accelerationChooser (distanceAccelerationMap:VehicleDistanceAccelerationMap) (distanceToObstacle:Distance) = 
            distanceAccelerationMap |> Map.tryPick (fun dist accel ->  if distanceToObstacle < dist then Some(accel) else None)  
        let compositeAccelerationChooser (choosers:seq<Distance->Acceleration option>) (distanceToObstacle:Distance) = 
            choosers |> Seq.map (fun choser -> choser distanceToObstacle) |> Seq.tryPick (fun accel -> accel)

       // let vehicleAccelerationUpdater2 (accelerationChooser:ByDistanceAccelerationChooser) (timeChange:TimeInterval) vehicle = 
    module VehicleUpdateWorflow = 
        type VehicleUpdateWorkflow = seq<VehicleUpdater>
        let apply (timeInterval:TimeInterval) (vehicle:Vehicle) (workflow: VehicleUpdateWorkflow) =
            workflow |> Seq.fold (fun vehicle vUpdater -> vUpdater timeInterval vehicle) vehicle
    module VehiclesUpdaters =     
        let update (vehicleUpdater: VehicleUpdater) (timeChange:TimeInterval) (vehicles:Vehicle seq) = 
            vehicles |> Seq.map (vehicleUpdater timeChange) 
        // updates vehicles on the same connection starting from vehicle with highest position on this connection
        let updateByPlacing (vehiclesUpdater: VehiclesUpdater) (timeChange:TimeInterval) (vehicles:Vehicle seq) = 
            vehicles |> Seq.groupBy (fun v -> v.Location.Placing) 
                     |> Seq.map ( (fun (_,vehicles) ->  vehicles |> Seq.sortByDescending (fun v->v.Location.CurrentProgress)) 
                                    >> (fun vehiclesOnSameConnection -> vehiclesOnSameConnection |> vehiclesUpdater timeChange))
                     |> Seq.fold (fun acc cur -> acc |> Seq.append cur) Seq.empty
                                                                

    