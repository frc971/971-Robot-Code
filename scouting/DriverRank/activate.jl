# A small helper to install all the dependencies on the scouting server.
using Pkg
Pkg.activate(ARGS[1])
Pkg.instantiate()
