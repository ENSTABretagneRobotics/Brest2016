#!/usr/bin/env  python
import cv2

# WEBCAM
camera = cv2.VideoCapture(0)

# DATA IMAGES
frame = cv2.imread('../data/boat0000.jpg')
mask = cv2.imread('../data/mask0000.jpg', 0)
res = cv2.bitwise_and(frame, frame, mask=mask)
# res = cv2.threshold(res, 10, 255, cv2.THRESH_BINARY)


# keep looping
while True:
    # grab the current frame
    # (grabbed, frame) = camera.read()

    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
    # if not grabbed:
    #     print 'no camera'
    #     break

    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask)
    cv2.imshow('res', res)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
