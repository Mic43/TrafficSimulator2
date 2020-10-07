namespace TrafficSimulator.Core
open DomainModel

module Collisions = 

    type Collidable = Vehicle of Vehicle

    type Collision =
        private { Collidables: Set<Collidable> }
        member this.getCollidables() = this.Collidables
    
    let create first second =
        if first = second
        then invalidArg "first" "Cannot create collision of object with itself"
        else { Collidables = [ first; second ] |> Set.ofList }

    let fromVehicles (first: Vehicle) (second: Vehicle) =
        create (Collidable.Vehicle first) (Collidable.Vehicle second)

    //let (|Collision|) collision =
    //    (collision.Collidables.MinimumElement, collision.Collidables.MaximumElement)

    let resolve allVehicles (collisions: Collision Set) =
        let flattenSet =
            collisions
            |> Set.fold (fun acc s -> acc |> (Set.union s.Collidables)) Set.empty

        let collidedVehiclesSet =
            flattenSet
            |> Set.map (fun collidable ->
                match collidable with
                | Vehicle v -> v.makeImmovable())
        (Set.difference allVehicles collidedVehiclesSet) |> Set.union collidedVehiclesSet
  