namespace TrafficSimulator.Tests

open System
open Xunit
open FsCheck.Xunit
open TrafficSimulator.Core.TrafficLights
open Microsoft.FSharp.Data.UnitSystems.SI.UnitSymbols
open FsCheck
open TrafficSimulator.Core.BaseTypes
open TrafficSimulator.Core.TrafficLights
open FsCheck
open TrafficSimulator.Core
open TrafficSimulator.Core.DomainModel

module PropertyBaseTests =

    type MyGenerators =
        static member PositiveDouble() =
            { new Arbitrary<float>() with
                member x.Generator =
                    Arb.Default.NormalFloat()
                    |> Arb.filter (fun t -> t.Get >= 0.0)
                    |> Arb.toGen
                    |> Gen.map float }

        static member TimeInterval() =
            { new Arbitrary<TimeInterval>() with
                member x.Generator =
                    MyGenerators.PositiveDouble().Generator
                    |> Gen.map (fun v -> TimeInterval.create (v * 1.0<s>)) }

        static member Fraction() =
            { new Arbitrary<Fraction>() with
                member x.Generator =
                    Arb.Default.PositiveInt()
                    |> Arb.toGen
                    |> Gen.map (fun v -> Fraction.create (1.0 / (float) v.Get)) }

        static member LightsGroup() =
            { new Arbitrary<LightsGroup>() with
                member x.Generator =
                    let lightsGen = Arb.generate<LightState>

                    let posListGen =
                        Arb.generate<ObjectLocation> |> Gen.nonEmptyListOf

                    let timeIntervalGen = Arb.generate<TimeInterval>

                    let lightSetGen =
                        Gen.map2 (fun positions state ->
                            positions
                            |> List.map (fun p -> { Location = p; State = state })
                            |> Set.ofList) posListGen lightsGen

                    Gen.map2 (fun ti ls -> Functions.ceateLightGroup ti ls) timeIntervalGen lightSetGen }

        static member LightSystem() =
            { new Arbitrary<LightSystem>() with
                member x.Generator =
                    let lightsGroupGen =
                        Arb.generate<LightsGroup>
                        |> Gen.two
                        |> Gen.filter (fun (first, second) ->
                            ((first.getLights () |> Set.map Functions.getState) =
                                (second.getLights () |> Set.map Functions.getState))
                            |> not)

                    let updateTimeGen = Arb.generate<float<s>>

                    Gen.map2 (fun (f, s) u ->
                        { FirstGroup = f
                          SecondGroup = s
                          LastUpdateTime = u }) lightsGroupGen updateTimeGen }

    [<Properties(Arbitrary = [| typeof<MyGenerators> |])>]
    module Lights =
        [<Property>]
        let ``Updating lightsystem does not changes lights position`` (lightSystem: LightSystem) ti =
            let updated =
                Functions.updateLightSystem ti lightSystem

            updated.getAllTrafficLights ()
            |> Set.map (fun x -> x.Location) = (lightSystem.getAllTrafficLights ()
                                                |> Set.map (fun x -> x.Location))

        [<Property>]
        let ``Updating lightsystem does not changes initial state of lights groups`` (lightSystem: LightSystem) ti =
            let updated =
                Functions.updateLightSystem ti lightSystem

            ([ updated.FirstGroup
               updated.SecondGroup ]
             |> List.map (fun x -> x.getInitialState ())) = ([ lightSystem.FirstGroup
                                                               lightSystem.SecondGroup ]
                                                             |> List.map (fun x -> x.getInitialState ()))

        //updated.FirstGroup.getInitialState() = lightSystem.FirstGroup.getInitialState()
        //&& updated.SecondGroup.getInitialState() = lightSystem.SecondGroup.getInitialState()

        [<Property>]
        let ``lights in same light group have same state after updating lightsystem`` (lightSystem: LightSystem) ti =
            let updated =
                Functions.updateLightSystem ti lightSystem

            [ updated.FirstGroup
              updated.SecondGroup ]
            |> List.map (Functions.getLights >> Set.map Functions.getState)
            |> List.forall (fun s -> s |> Set.count <= 1)

        [<Property>]
        let ``Updating lightsystem increases LastUpdateTime`` (lightSystem: LightSystem) ti =
            let updated =
                Functions.updateLightSystem ti lightSystem

            updated.LastUpdateTime = lightSystem.LastUpdateTime + ti.Value

        [<Property>]
        let ``lights in different light groups should have opposite state after updating lightsystem`` (lightSystem: LightSystem)
                                                                                                       ti
                                                                                                       =
            let updated =
                Functions.updateLightSystem ti lightSystem

            updated.FirstGroup.getState () = updated.SecondGroup.getState().Opposite
