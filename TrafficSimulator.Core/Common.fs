module Common
    open Option
    [<Struct>]
    type OptionalBuilder =
      member _.Bind(opt, binder) =
        Option.bind binder opt
      member _.Return(value) =
        Some value
    let optional = OptionalBuilder()