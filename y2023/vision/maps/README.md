## Map conventions
Targets have positive Z axis pointing into the board, positive X to the right when looking at the board,
and positive Y is down when looking at the board.
This means that you will get an identity rotation from the camera to target
frame when the target is upright, flat, and centered in the camera's view for an upright camera.
For 2023 cameras, the target needs to be upside down for the identity rotation because the cameras are flipped.

## Target mapping with box of pis
### Setup
- Deploy code to pi2, pi3, and pi6 (imu) on the box of pis
- Choose to map red side (id 1, 2, 3) or blue side (id 5, 6, 7). Ignore human player april tags.
- Open live foxglove windows on pi2 and pi3 to display what they're seeing
- Start logging on IMU pi
### Capturing the targets
- Start at first target, get view with one pi seeing this target and other pi seeing the second target, stay stationary for a bit
- Then give a view of one pi seeing both targets for a bit. Then go back a bit and pause at both views for a bit more
- Come back forward and with the view of one pi seeing each of the two targets, move up and down a bit.
- Move to second pair of targets and repeat this motion. Then do the whole thing in reverse, ending back up where you started
### Post-capture
- Stop logging on IMU and offload the latest log
- Run target mapping to see where the targets *actually* are!
