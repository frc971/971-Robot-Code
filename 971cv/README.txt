This is a working computer vision target recognizer for the FRC 2013 game.

Run it on 2013-test-images/*.jpg.

TODO: Capture 640x480 images with the robot's camera (Logitech C210), green LED
      light ring, and the shutter speed turned down as in 2012.
      Also turn off auto whitebalance.

    src/org/frc971/DebugCanvas.java     -- displays intermediate frames for tuning
    src/org/frc971/Recognizer.java      -- the vision recognizer interface
    src/org/frc971/Recognizer2013.java  -- the UI independent vision recognizer
    src/org/frc971/VisionTuner.java     -- a Java UI for testing and tuning the recognizer

TODO: Connect the recognizer to the robot camera (e.g. using ffmpeg).
TODO: Get the relevant robot measurements: Camera height, setback, pitch,
      horizontal FOV, and vertical FOV.
TODO: Compute target distance, elevation, and azimuth, and send them to the
      robot's control loops.
TODO: Pass through a timestamp with each frame.
TODO: Wean off of WPIJavaCV and its DaisyExtensions.
