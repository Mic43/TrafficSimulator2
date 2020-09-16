namespace TrafficSimulator.Core

open Microsoft.FSharp.Data.UnitSystems.SI.UnitSymbols
open System
open Common
open BaseTypes

module QuadraticBezier =

    type QuadraticBezier =
        private { controlP: Position
                  SegmentsCache: List<float> }
        member this.controlPoint = this.controlP
        member this.segmentsCache = this.SegmentsCache

        member this.calculate t startP endP =
            match this.controlP with
            | Position2d controlP ->
                let a = (1.0 - t) * (1.0 - t)
                let p0 = startP
                let p1 = controlP
                let p2 = endP

                { X = p1.X + a * (p0.X - p1.X) + t * t * (p2.X - p1.X)
                  Y = p1.Y + a * (p0.Y - p1.Y) + t * t * (p2.Y - p1.Y) }

    let calculateLenght p0 p1 p2 =
        match (p0, p1, p2) with
        | (Position2d p0, Position2d p1, Position2d p2) ->
            let a =
                { X = p0.X - 2.0 * p1.X + p2.X
                  Y = p0.Y - 2.0 * p1.Y + p2.Y }

            let b =
                { X = 2.0 * p1.X - 2.0 * p0.X
                  Y = 2.0 * p1.Y - 2.0 * p0.Y }

            let A = 4.0 * (a.X * a.X + a.Y * a.Y)
            let B = 4.0 * (a.X * b.X + a.Y * b.Y)
            let C = b.X * b.X + b.Y * b.Y

            let Sabc = 2.0 * sqrt (A + B + C)
            let A_2 = sqrt (A)
            let A_32 = 2.0 * A * A_2
            let C_2 = 2.0 * sqrt (C)
            let BA = B / A_2

            (A_32
             * Sabc
             + A_2 * B * (Sabc - C_2)
             + (4.0 * C * A - B * B)
               * log ((2.0 * A_2 + BA + Sabc) / (BA + C_2)))
            / (4.0 * A_32)

    let create startP controlPoint endP (samplesCount: int) =
        let segmentLen =
            (calculateLenght startP controlPoint endP)
            / (float) samplesCount

        let v1 =
            (startP.mul 2.0)
            |> Position.add (controlPoint.mul -4.0)
            |> Position.add (endP.mul 2.0)

        let v2 =
            (startP.mul -2.0)
            |> Position.add (controlPoint.mul 2.0)

        let divider (t: float) =
            Position.len (v1 |> (Position.mul t) |> Position.add v2)

        let mutable segments = Map.empty

        let mutable t = 0.0
        for i = 0 to samplesCount do
            t <- t + segmentLen / (divider t)
            segments <- segments.Add(Fraction.create ((float i) / (float samplesCount)), t)

        { controlP = controlPoint
          SegmentsCache =
              segments
              |> Map.toList
              |> List.map (fun (_, v) -> v) }
