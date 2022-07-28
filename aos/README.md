See [Top level README](../README.md) for overall information

### `aos_dump`

For examples on viewing events and logging, see the [README.md](./events/README.md) file in `aos/events`


### `aos_graph_nodes`

This provides an easy way to visualize the connections and messages being passed between nodes in our system.

Run with `--help` for more on usage.  To see a graphical visualization, pipe the output through `dot` using an X11 display window:
```
aos_graph_nodes | dot -Tx11
```

### Rust

AOS has experimental rust support. This involves creating Rust wrappers for all of the relevant
C++ types. There must be exactly one wrapper for each type, or you will get confusing errors about
trying to convert Rust types with very similar names (in different crates though) when you try
using them together. To standardize this, we have some conventions.

We use autocxx to generate the raw wrappers. Sometimes autocxx needs tweaked C++ signatures to
generate usable Rust bindings. These go in a separate C++ file with a `_for_rust` suffix, and have
functions with `ForRust` suffixes.

We want to pass around pointers and references to the autocxx-generated flatbuffers types so we can
create byte slices to use with the Rust versions, but we ignore many of the flatbuffers types needed
to wrap individual methods. Some of them are tricky to wrap.

Converting between the autocxx-generated and rustc-generated flatbuffers types is tricky. The Rust
flatbuffers API is based on slices, but the C++ API that autocxx is wrapping just uses pointers. We
can convert from a Rust flatbuffer to its C++ equivalent pretty easily, but going the other way
doesn't work. To maximize flexibility, each C++ wrapper module exposes APIs that take
autocxx-generated types and provide convenient conversions for the types belonging to that module.
Flatbuffers returned from C++ by value (typically in a `aos::Flatbuffer`) get returned as Rust
`aos_flatbuffers::Flatbuffer` objects, while ones being returned from C++ by pointer (or reference)
are exposed as the autocxx types.

For the file `aos/xyz.fbs`, Rust wrappers go in `aos/xyz.rs`. The Rust flatbuffers generated
code will be in `aos/xyz_fbs.rs`.

For the file `aos/abc.h`, Rust wrappers go in `aos/abc.rs`. These wrappers may be more sophisticated
than simple unsafe wrappers, but they should avoid adding additional functionality. Any additional
C++ code goes in `aos/abc_for_rust.h`/`aos/abc_for_rust.cc`.

All Rust functions intended to be called from other files gets exported outside of the `ffi`
module. In some cases, this is just giving the raw autocxx wrappers a new name. In other cases,
these wrappers can attach lifetimes etc and be safe. This makes it clear which functions and
types are being exported, because autocxx generates a lot of wrappers. Do not just make the
entire `ffi` module, or any of its submodules, public.

Rust modules map to Bazel rules. This means we end up lots of Rust modules. We name them like
`aos_events_event_loop` for all the code in `aos/events/event_loop.rs`.

### NOTES

Some functions need to be in separate translation units in order for them to be guaranteed to work. As the C standard says,

> Alternatively, an implementation might perform various optimizations
> within each translation unit, such that the actual semantics would
> agree with the abstract semantics only when making function calls
> across translation unit boundaries. In such an implementation, at the
> time of each function entry and function return where the calling
> function and the called function are in different translation units,
> the values of all externally linked objects and of all objects
> accessible via pointers therein would agree with the abstract
> semantics. Furthermore, at the time of each such function entry the
> values of the parameters of the called function and of all objects
> accessible via pointers therein would agree with the abstract
> semantics. In this type of implementation, objects referred to by
> interrupt service routines activated by the signal function would
> require explicit specification of volatile storage, as well as other
> implementation-defined restrictions.

### FILES  <TODO: I believe these are no longer correct>
- `config/` has some configuration files
  - `aos.conf` (currently in `aos` folder) has directions for setting up resource limits so you can run the code on any linux machine (the directions are there to keep them with the file)
  - `setup_rc_caps.sh` (currently in `aos` folder) is a shell script (you have to run as root) that lets you run the realtime code without installing aos.conf for an individual file
  - `starter` is an init.d file
    - install it by putting it in /etc/init.d an running `update-rc.d starter defaults`
    - restart it by running `invoke-rc.d starter restart` (doesn't always work very well...)
  - the .config files are for building linux kernels
