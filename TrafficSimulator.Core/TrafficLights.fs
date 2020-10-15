namespace TrafficSimulator.Core

open Microsoft.FSharp.Data.UnitSystems.SI.UnitSymbols
open Common
open BaseTypes
open DomainModel
open DomainFunctions

module TrafficLights =
    type LightState =
        | Red
        | Green
        member this.Opposite =
            (if this = LightState.Red then LightState.Green else LightState.Red)

    type TrafficLight =
        { Location: ObjectLocation
          State: LightState }

    type LightsGroup =
        private { Lights: TrafficLight Set
                  InitialState: LightState
                  GreenLightInterval: TimeInterval }
        member this.changeLightTo lightState =
            { this with
                  Lights =
                      this.Lights
                      |> Set.map (fun l -> { l with State = lightState }) }

        member this.getLights() = this.Lights
        member this.getInitialState() = this.InitialState
        member this.getState() = 
            assert (this.Lights.IsEmpty |> not)
            this.Lights.MinimumElement.State
    type LightSystem =
        { FirstGroup: LightsGroup
          SecondGroup: LightsGroup
          LastUpdateTime: float<s> }
        member this.CycleTimeSpan =
            let v =
                TimeInterval.tryCreate
                    (this.FirstGroup.GreenLightInterval.Value
                     + this.SecondGroup.GreenLightInterval.Value)

            assert v.IsSome
            v.Value

        member this.getAllTrafficLights() =
            this.FirstGroup.Lights
            |> Set.union this.SecondGroup.Lights

    module Functions =
        let getLights (lightsGroup:LightsGroup) = lightsGroup.getLights()
        let getState (trafficLight:TrafficLight) = trafficLight.State

        let createRedLight location =
            { Location = location
              State = LightState.Red }

        let createGreenLight location =
            { Location = location
              State = LightState.Green }

        let tryCreateLightGroup greenLightInterval lightsSet =
            let lightStates =
                (lightsSet |> Set.map getState)

            if lightStates |> Set.count = 1 |> not then
                None
            else
                let onlyElement =
                    lightStates
                    |> Set.toList
                    |> List.tryExactlyOne
                    |> Option.defaultValue LightState.Green

                Some
                    ({ Lights = lightsSet
                       InitialState = onlyElement
                       GreenLightInterval = greenLightInterval })
        let ceateLightGroup greenLightInterval lightsSet =
                  tryCreateLightGroup greenLightInterval lightsSet
                  |> Option.defaultWith (fun () -> invalidArg "value" "Not possible to create such light group")
        let createSimpleLightSystem greenLightInterval
                                    redLightInterval
                                    (lane1: Connection)
                                    (lane2: Connection)
                                    (location: Fraction)
                                    =
            if not
                (lane1.EndId = lane2.StartId
                 && lane1.StartId = lane2.EndId) then
                None
            else
                let group1 =
                    (seq {
                        createGreenLight
                            { Placing = lane1
                              CurrentProgress = location }

                        createGreenLight
                            { Placing = lane2
                              CurrentProgress = location.getSymmetrical }
                     }
                     |> Set.ofSeq)
                    |> tryCreateLightGroup greenLightInterval

                let fakegroup =
                    Set.empty |> tryCreateLightGroup redLightInterval

                optional {
                    let! g1 = group1
                    let! fg = fakegroup

                    return
                        { FirstGroup = g1
                          SecondGroup = fg
                          LastUpdateTime = 0.0<s> }
                }

        let createCrossingLightSystem (firstGroup: Connection Set)
                                      (secondGroup: Connection Set)
                                      greenLightInterval1
                                      greenLightInterval2
                                      (connectionLenProvider:ConnectionLenghtProvider)                                      
                                      =
            optional {
                let! fg =
                    firstGroup
                    |> Set.map (fun c -> c.EndId)
                    |> Set.toList
                    |> List.tryExactlyOne

                let! sg =
                    secondGroup
                    |> Set.map (fun c -> c.EndId)
                    |> Set.toList
                    |> List.tryExactlyOne

                if not (fg = sg) then
                    return None
                else                   
                    let lightsDistance = 1.0<m>
                    let getFraction len= (Fraction.tryFromDistance (len - lightsDistance) len) |> Option.defaultValue Fraction.zero
                    //let! pos connection = (connectionLenProvider connection)
                    let! g1 =
                        tryCreateLightGroup
                            greenLightInterval1
                            (firstGroup
                             |> Set.map ( fun c -> (c,(connectionLenProvider c)))
                             |> Set.map (fun (c,len) -> createGreenLight { Placing = c; CurrentProgress = (getFraction len)}))

                    let! g2 =
                        tryCreateLightGroup
                            greenLightInterval2
                            (secondGroup
                             |> Set.map ( fun c -> (c,(connectionLenProvider c)))
                             |> Set.map (fun (c,len) -> createRedLight  { Placing = c; CurrentProgress = (getFraction len)}))

                    return
                        Some
                            { FirstGroup = g1
                              SecondGroup = g2
                              LastUpdateTime = 0.0<s> }
            }
            |> Option.flatten

        let updateLightSystem (timeInterval: TimeInterval) lightSystem =
            let timePassed =
                lightSystem.LastUpdateTime + timeInterval.Value

            let temp =
                timePassed / lightSystem.CycleTimeSpan.Value

            let rest =
                TimeInterval.tryCreate
                    ((temp - float (int temp))
                     * lightSystem.CycleTimeSpan.Value)

            if rest.IsNone
            then failwith "cannot create timespan les than zero lenght"

            let firstGrouplightState =
                if rest.Value < lightSystem.FirstGroup.GreenLightInterval
                then LightState.Green
                else LightState.Red

            let updatedFirstGroup =
                lightSystem.FirstGroup.changeLightTo firstGrouplightState

            let updatedSecondGroup =
                lightSystem.SecondGroup.changeLightTo firstGrouplightState.Opposite

            { lightSystem with
                  FirstGroup = updatedFirstGroup
                  SecondGroup = updatedSecondGroup
                  LastUpdateTime = timePassed }
