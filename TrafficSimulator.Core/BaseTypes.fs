namespace TrafficSimulator.Core

open Microsoft.FSharp.Data.UnitSystems.SI.UnitSymbols
open System

module BaseTypes =

    type CrossingId = CrossingId of int
    type VehicleId = VehicleId of int

    [<Struct>]
    type Position2d = { X: float; Y: float }

    type Position =
        | Position2d of Position2d
        member this.mul scalar =
            match this with
            | Position2d pos ->
                Position.Position2d
                    { X = pos.X * scalar
                      Y = pos.Y * scalar }

        member this.add other =
            match (this, other) with
            | (Position2d pos, Position2d other) ->
                Position.Position2d
                    { X = pos.X + other.X
                      Y = pos.Y + other.Y }

    let private lerp x y t = x + (y - x) * t

    module Position =
        let len (point: Position) =
            match point with
            | Position2d p -> sqrt (p.X * p.X + p.Y * p.Y)

        let add (p1: Position) (p2: Position) = p1.add p2
        let mul scalar (p: Position) = p.mul scalar
        let distance (point: Position) (point2: Position) = len (point2.add (point.mul -1.0))

        let lerp (v1: Position2d) (v2: Position2d) (t) =
            { X = lerp v1.X v2.X t
              Y = lerp v1.Y v2.Y t }

    type Distance = float<m>
    type Speed = float<m / s>
    type Acceleration = float<m / (s * s)>

    type TimeInterval =
        | TimeInterval of float<s>
        member this.Value =
            let (TimeInterval v) = this
            v   
        static member ( + ) (left: TimeInterval, right: TimeInterval) =
            TimeInterval (left.Value + right.Value)

    module TimeInterval =
        let tryCreate value =
            if value >= 0.0<s> then Some(TimeInterval value) else None
        let create value = 
            (tryCreate value) |> Option.defaultWith (fun () -> failwith "cannot create neative interval")
        let zero() = create 0.0<s> 
    type Fraction =
        private
        | Fraction of float
        member this.Value =
            let getValue (Fraction f) = f
            getValue this

        member this.getSymmetrical = Fraction(1.0 - this.Value)

    module Fraction =
        let tryCreate (value: float) =
            if ((Math.Abs value) <= 1.0) then Some(Fraction value) else None

        let create (value: float) =
            tryCreate value
            |> Option.defaultWith (fun () -> invalidArg "value" "Not possible to create fraction not between -1 and 1")

        let zero = Fraction 0.0
        let one = Fraction 1.0
        let tryFromDistance (distance: Distance) (connectionLenght: float<m>) = tryCreate (distance / connectionLenght)
        let fromDistance (distance: Distance) (connectionLenght: float<m>) = create (distance / connectionLenght)
        let toDistance (connectionLenght: Distance) (Fraction value) = value * connectionLenght

        let distanceOnSameConnecton (progressFrom: Fraction) (progressTo: Fraction) connectionLenght =
            let dist =
                tryCreate (progressTo.Value - progressFrom.Value)
                |> Option.defaultWith (fun () -> failwith "Not possible to create fraction not between -1 and 1")

            toDistance connectionLenght dist
