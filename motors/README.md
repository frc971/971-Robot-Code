# Driver station
## Setup
1. Install Teensy Loader [here](https://www.pjrc.com/teensy/loader_mac.html).

## Make changes
1. Most of the code lives in `/motors/driver_station*`.

## Deploy
1. Build driver station. This outputs a file `bazel-bin/motors/driver_station.hex`.
    ```shell
    bazel build -c opt --config=cortex-m4f //motors:driver_station
    ```

2. Send the file to your local development machine.
    ```shell
    scp <user@build.frc971.org>:917-Robot-Code/bazel-bin/motors/driver_station.hex ~/Downloads/driver_station.hex
    ```

3. Open hex file in Teensy loader. Press button on Teensy.

## Swerve
1. To allow a swerve controller to work, a jumper should be in the BTN17 spot (view schematics below).

## References
- Electrical schematics [here](https://github.com/frc971/electrical/blob/main/robots/2023/driver-station/DriverStation-8Apr2023.pdf).
- Joystick adc pin mapping[here](https://github.com/frc971/electrical/blob/main/robots/2025/Driver%20Station/Harness%20Drawings/TX16S-Joystick-Input.png).
