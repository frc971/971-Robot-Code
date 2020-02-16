import cv2
# Open the device at the ID X for /dev/videoX
cap = cv2.VideoCapture(2)

#Check whether user selected camera is opened successfully.
if not (cap.isOpened()):
    print("Could not open video device")
    quit()

while (True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    exp = cap.get(cv2.CAP_PROP_EXPOSURE)
    print("Exposure:", exp)
    # Display the resulting frame
    cv2.imshow('preview', frame)

    #Waits for a user input to quit the application
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
