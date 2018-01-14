#!/bin/bash
# Deploy to the driver station side of the pistol grip.

bazel build --cpu=cortex-m4f -c opt //motors/pistol_grip:drivers_station.hex && bazel run //motors/teensy_loader_cli -- --mcu=mk64fx512 -s $(readlink -f bazel-bin/motors/pistol_grip/drivers_station.hex)
