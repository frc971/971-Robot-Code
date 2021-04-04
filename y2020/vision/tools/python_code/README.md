# Notes on running the SIFT matching code

Here are some of the key modules:

## Camera Definition
  * Class defined in camera_definition.py
    * Includes:
      * Camera Intrinsic
      * Camera Extrinsic
      * Team number & node name
    * We create a separate camera definition for each camera (on each robot)
    * For now, they're all identical, but we need a mechanism for having each be unique, esp. for the extrinsics

## Target Definition
  * Class defined in target_definition.py
    * For each target (e.g., Power Port Red, Power Port Blue, Loading Station Red, etc), we define both an ideal target (e.g., the CAD / graphic version, with known 3D point locations) and a training target (an image from our cameras in the environment we're expecting).
      * KEY IDEA: We use the ideal target info (including 3D locations of points) to determine the 3D locations of the keypoints in the training target

    * The following are parts of the TargetData class:
      * image
      * List of polygons which define flat surfaces on the ideal target
        * List of corresponding 3D points for the polygons, so we can automatically extract 3D locations of the keypoints (via interpolation)
      * Keypoint List of extracted keypoints from the target image (ideal & training)
        * Descriptor list (which is used to match features)
      * (Ideal) Target point position and rotation
      * (Ideal) Target point 2D-- location of a special point in the target used to help visualize accuracy of matches
      * Target point radius (used to help size things to check on distance)
      * Keypoint List 3d of the 3D locations of each of the keypoints
        * These are computed based on the 3D polygon definition
        * These are 3D with respect to our global origin
        * This is used to determine our pose as we do th ematching

## Header definition of target data

  * Creating the config file
    * The output of all this is a sift_training_data.h file that gets used in other parts of the code (e.g., //y2020/vision:camera_reader)
    * By building "run_load_training_data", you should get this file generated

## Testing / debugging
  * There's a couple tests that can be run through bazel
  * image_match_test.py is a good way to see what's being matched
    * You have to specify and select the correct query_image to test on
