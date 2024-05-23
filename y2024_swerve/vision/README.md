How to use the extrinsic calibration for camera-to-camera calibration

This all assumes you have cameras that have been intrinsically
calibrated, and that pi1 has a valid extrinsic calibration (from robot
origin / IMU to the pi1 camera).

It by default will compute the extrinsics for each of the other cameras (pi2,
pi3, pi4) relative to the robot origin (IMU origin).

Steps:
* Place two Aruco Diamond markers about 1 meter apart
  (center-to-center) at a height so that they will be in view of the
  cameras when the robot is about 1-2m away
* Start with the robot in a position that both markers are fully in
  view by one camera
* Enable logging for all cameras
* Rotate the robot in place through a full circle, so that each camera sees both tags, and at times just one tag.
* Stop the logging and copy the files to your laptop
* Run the calibration code on the resulting files, e.g.,
  * `bazel run -c opt //y2023/vision:calibrate_multi_cameras -- /data/PATH_TO_LOGS --team_number 971 --target_type charuco_diamond
  * Can add "--visualize" flag to see the camera views and marker detections
  * I've sometimes found it necessary to add the "--skip_missing_forwarding_entries" flag-- as guided by the logging messages
* From the output, copy the calculated ("Updated") extrinsic into the
  corresponding calibration file for the right robot and camera in the
  calib_files directory
