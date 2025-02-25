# Camera Calibration Tutorial

## Camera Focus
- **Command**: `./bin/camera_focus --camera_channel=/camera0`

### `--camera_channel`
- **Description**: The flatbuffer channel of the camera
- **Instructions**: Display camera footage through Foxglove and wave your hand in front of the camera to find which camera is connected to which channel.

---

## Calibrate Cameras Intrinsics

### Setup X Server
- **Linux**: Should be already installed by default.
- **MacOS**: Download and install [XQuartz](http://xquartz.org/).

### SSH into the Box of Orins
- Set subnet mask to `255.255.255.0` (or `/24`).
- ForwardX11 is the same as passing in the -X flag when sshing
#### SSH Config Examples:
##### Orin 1
```plaintext
Host 7971
  HostName 10.79.71.101
  User pi
  ForwardX11 yes
```

##### Orin 2
```plaintext
Host 7972
  HostName 10.79.71.102
  User pi
  ForwardX11 yes
```

### Calibration Process
1. Ensure the cameras are not moving.
2. SSH into the Orin and run the following command in the `bin` directory to start the calibration:
   Command:
   ```bash
   ./intrinsics_calibration --base_intrinsics {base_intrinsics} --camera_id {camera_id} --channel={channel} --cpu_name {cpu_name} --visualize
   ```
   Example:
   ```bash
   ./intrinsics_calibration --base_intrinsics ~/calibration_orin-971-1_cam-24-00_8parameter.json --camera_id 24-14 --channel=/camera1 --cpu_name orin-7971-1 --visualize
   ```

### Flags
- `--base_intrinsics`
  - **Description**: The starting 8 parameters for the calibrator.
  - **Location**: Can be found in `___`.

- `--camera_id`
  - **Description**: The title for the camera you are calibrating.
  - **Format**: Usually in the format `year-camera_number`.
  - **Note**: The camera itself should be labeled with the camera ID. This flag is just a title, so technically, anything can be entered.

- `--channel`
  - **Description**: The channel to read the camera images from.
  - **Instructions**: Use Foxglove to determine which camera uses which channel.

- `--cpu_name`
  - **Description**: Name of the processor being used.
  - **Format**: `processor type, team number, processor id`.
  - **Examples**:
    - Box of Orin GPU #1: `orin-7971-1`
    - Box of Orin GPU #2: `orin-7971-2`
  - **Note**: Ensure the `cpu_name` matches the value set in `aos_config.json`.

- `--visualize`
  - **Description**: Required flag to visualize the calibration process.
```
