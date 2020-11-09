See [Top level README](../README.md) for overall information

### `aos_dump`

For examples on viewing events and logging, see the [README.md](./events/README.md) file in `aos/events`


### `aos_graph_nodes`

This provides an easy way to visualize the connections and messages being passed between nodes in our system.

Run with `--help` for more on usage.  To see a graphical visualization, pipe the output through `dot` using an X11 display window:
```
aos_graph_nodes | dot -Tx11
```

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
