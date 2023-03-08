# Driver ranking parsing script

This directory contains the script that parses the raw data that the scouts
collect on driver rankings and gives each team a score.

## Deployment

Whenever the Julia environment is set up for the first time or whenever the
dependencies are updated, the Julia package needs to be redeployed. This is a
separate step from the scouting server deployment because the Julia runtime is
huge.

```console
$ bazel run //scouting/DriverRank:deploy -- --host scouting
```
