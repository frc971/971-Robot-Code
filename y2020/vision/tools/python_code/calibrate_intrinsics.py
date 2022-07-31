import cv2
import cv2.aruco
import datetime
import json
from json import JSONEncoder
import numpy as np
import os
import time


# From: https://pynative.com/python-serialize-numpy-ndarray-into-json/
class NumpyArrayEncoder(json.JSONEncoder):

    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        else:
            return super(NumpyArrayEncoder, self).default(obj)


def get_robot_info(hostname):
    hostname_split = hostname.split("-")
    if hostname_split[0] != "pi":
        print("ERROR: expected hostname to start with pi!  Got '%s'" %
              hostname)
        quit()

    team_number = int(hostname_split[1])
    node_name = hostname_split[0] + hostname_split[2]
    return node_name, team_number


USE_LARGE_BOARD = True

if USE_LARGE_BOARD:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard_create(12, 9, .06, .045, dictionary)
    img = board.draw((200 * 12, 200 * 9))
else:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard_create(11, 8, .015, .011, dictionary)
    img = board.draw((200 * 11, 200 * 8))

#Dump the calibration board to a file
#cv2.imwrite('charuco.png', img)

#Start capturing images for calibration
CAMERA_INDEX = 0  # Capture from /dev/videoX, where X=CAMERA_INDEX
cap = cv2.VideoCapture(CAMERA_INDEX)

allCorners = []
allIds = []
capture_count = 0
MIN_IMAGES = 50

while (capture_count < MIN_IMAGES):

    ret, frame = cap.read()
    assert ret, "Unable to get image from the camera at /dev/video%d" % CAMERA_INDEX
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = cv2.aruco.detectMarkers(gray, dictionary)
    aruco_detect_image = frame.copy()

    if len(res[0]) > 0 and len(res[1]) > 0:
        cv2.aruco.drawDetectedMarkers(aruco_detect_image, res[0], res[1])

    # Display every image to let user trigger capture
    cv2.imshow('frame', aruco_detect_image)
    keystroke = cv2.waitKey(1)

    if keystroke & 0xFF == ord('q'):
        break
    elif keystroke & 0xFF == ord('c'):
        print("Asked to capture image")
        if len(res[0]) == 0 or len(res[1]) == 0:
            # Can't use this image
            continue

        res2 = cv2.aruco.interpolateCornersCharuco(res[0], res[1], gray, board)
        if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3:
            capture_count += 1
            charuco_detect_image = frame.copy()
            allCorners.append(res2[1])
            allIds.append(res2[2])
            print("Capturing image #%d" % capture_count)
            cv2.aruco.drawDetectedCornersCharuco(charuco_detect_image, res2[1],
                                                 res2[2])

            cv2.imshow('frame', charuco_detect_image)
            cv2.waitKey(1000)
            # TODO<Jim>: Should log image to disk

#Calibration fails for lots of reasons. Release the video if we do
try:
    imsize = gray.shape
    cal = cv2.aruco.calibrateCameraCharuco(allCorners, allIds, board, imsize,
                                           None, None)
    print("Calibration is:\n", cal)
    print("Reproduction error:", cal[0])
    if (cal[0] > 1.0):
        print("REPRODUCTION ERROR NOT GOOD")
    print("Calibration matrix:\n", cal[1])
    print("Distortion Coefficients:\n", cal[2])
except:
    print("Calibration failed")
    cap.release()
    quit()

hostname = os.uname()[1]
date_str = datetime.datetime.today().strftime("%Y-%m-%d-%H-%M-%S")
node_name, team_number = get_robot_info(hostname)
numpyData = {
    "hostname": hostname,
    "node_name": node_name,
    "team_number": team_number,
    "timestamp": date_str,
    "camera_matrix": cal[1],
    "dist_coeffs": cal[2]
}
encodedNumpyData = json.dumps(
    numpyData, cls=NumpyArrayEncoder)  # use dump() to write array into file

# Write out the data
calib_file = open("cam_calib_%s_%s.json" % (hostname, date_str), "w")
calib_file.write(encodedNumpyData)
calib_file.close()

cap.release()
cv2.destroyAllWindows()
