import cv2 as cv
import numpy as np

cap = cv.VideoCapture(0)
cv.namedWindow('frame',cv.WINDOW_NORMAL)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    #ret, thresh = cv.threshold(gray, 180, 255, cv.THRESH_BINARY)

    #frame[thresh != 255] = 0
    #frame[thresh == 255] = 255

    # Display the resulting frame
    cv.imshow('frame', frame)
    if cv.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
