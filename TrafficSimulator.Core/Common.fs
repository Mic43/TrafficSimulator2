module Common

open Option

[<Struct>]
type OptionalBuilder =
    member _.Bind(opt, binder) = Option.bind binder opt
    member _.Return(value) = Some value
    member _.ReturnFrom(value) = value
    member _.Zero() = None

let optional = OptionalBuilder()

module Stateful =
    type Stateful<'TState, 'TResult> = Stateful of ('TState -> ('TState * 'TResult))
    type Stateful<'TState> = Stateful<'TState, unit>

    let execute state (Stateful stateful) = stateful state
    let fromAction<'TState> (f: 'TState -> 'TState) = Stateful(fun state -> (f state, ()))
    let ret v = Stateful(fun state -> (state, v))

    let bind<'TState, 'TResult, 'TResult2> (binder: 'TResult -> Stateful<'TState, 'TResult2>)
                                           (stateful: Stateful<'TState, 'TResult>)
                                           =
        let tmp = (fun (s: 'TState) -> execute s stateful)

        let x =
            fun s ->
                let (s, r) = (tmp s)
                execute s (r |> binder)

        Stateful x

    [<Struct>]
    type StatefulBuilder =
        member _.Bind(opt, binder) = bind binder opt
        member _.Return(value) = ret value
        member _.ReturnFrom(value) = value
    let stateful = StatefulBuilder()
module Reader = 
    type Reader<'TState,'TOutput> = Reader of ('TState->'TOutput) 
    let execute s (Reader reader) = reader s
    let ret v = Reader (fun _ -> v)
    let bind binder (reader:Reader<'TState,'TOutput>) = 
        let a = fun s -> execute s reader
        let b = fun s -> (a s) |> binder
        let c = fun s -> (b s) |> execute s
        Reader c

    [<Struct>]
    type ReaderBuilder =
          member _.Bind(opt, binder) = bind binder opt
          member _.Return(value) = ret value
          member _.ReturnFrom(value) = value
    let reader = ReaderBuilder()
        //let reader2 = ret (fun s-> binder (execute s reader))
        //reader2