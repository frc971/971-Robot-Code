import time
import cv2
import cv2.aruco as A
import numpy as np

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(11, 8, .015, .011, dictionary)
img = board.draw((200 * 11, 200 * 8))

#Dump the calibration board to a file
cv2.imwrite('charuco.png', img)

#Start capturing images for calibration
CAMERA_INDEX = 2
cap = cv2.VideoCapture(CAMERA_INDEX)

allCorners = []
allIds = []
capture_count = 0
while (capture_count < 50):

    ret, frame = cap.read()
    assert ret, "Unable to get image from the camera at /dev/video%d" % CAMERA_INDEX
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = cv2.aruco.detectMarkers(gray, dictionary)
    aruco_detect_image = frame.copy()

    if len(res[0]) > 0:
        cv2.aruco.drawDetectedMarkers(aruco_detect_image, res[0], res[1])

    cv2.imshow('frame', aruco_detect_image)
    keystroke = cv2.waitKey(1)
    if keystroke & 0xFF == ord('q'):
        break
    elif keystroke & 0xFF == ord('c'):
        print("Res:", len(res[0]), res[1].shape)
        res2 = cv2.aruco.interpolateCornersCharuco(res[0], res[1], gray, board)
        if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3:
            capture_count += 1
            charuco_detect_image = frame.copy()
            allCorners.append(res2[1])
            allIds.append(res2[2])
            print("Res2: ", res2[1].shape, res2[2].shape)
            cv2.aruco.drawDetectedCornersCharuco(charuco_detect_image, res2[1],
                                                 res2[2])
            cv2.imshow('frame', charuco_detect_image)
            cv2.waitKey(1000)
            # TODO: Should log image to disk
            print("Captured image #", capture_count)

imsize = gray.shape

#Calibration fails for lots of reasons. Release the video if we do
try:
    cal = cv2.aruco.calibrateCameraCharuco(allCorners, allIds, board, imsize,
                                           None, None)
    #print("Calibration is:\n", cal)
    print("Reproduction error:", cal[0])
    if (cal[0] > 1.0):
        print("REPRODUCTION ERROR NOT GOOD")
    # TODO<jim>: Need to save these out in format that can be used elsewhere
    print("Calibration matrix:\n", cal[1])
    print("Distortion Coefficients:\n", cal[2])
except:
    print("Calibration failed")
    cap.release()

cap.release()
cv2.destroyAllWindows()
