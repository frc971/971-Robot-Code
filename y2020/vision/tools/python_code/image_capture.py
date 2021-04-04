#!/usr/bin/python3

import cv2
import datetime
import sys

import gflags
import glog

FLAGS = gflags.FLAGS


def main(argv):
    # Open the device at the ID X for /dev/videoX
    # NOTE: if camera_reader is running, it will need to be stopped
    # since it will have control of the camera.
    CAMERA_INDEX = 0
    cap = cv2.VideoCapture(CAMERA_INDEX)

    # Check whether user selected camera is opened successfully.
    if not (cap.isOpened()):
        print("Could not open video device /dev/video%d" % CAMERA_INDEX)
        return

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        exp = cap.get(cv2.CAP_PROP_EXPOSURE)
        #print("Exposure:", exp)
        # Display the resulting frame
        cv2.imshow('preview', frame)

        # Wait for a user input to capture image or quit the application
        keystroke = cv2.waitKey(1)

        if keystroke & 0xFF == ord('q'):
            break
        elif keystroke & 0xFF == ord('c'):
            image_name = datetime.datetime.today().strftime(
                "capture-%Y-%m-%d-%H-%M-%S.png")
            print("Capturing image as %s" % image_name)
            cv2.imwrite(image_name, frame)

    # When everything's done, release the camera
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    sys.exit(main(argv))
