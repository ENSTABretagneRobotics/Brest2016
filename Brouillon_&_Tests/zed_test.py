#!/usr/bin/env python
import cv2
import numpy as np


def t1():
    cap = cv2.VideoCapture(0)

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Our operations on the frame come here
        imgL, imgR = np.array_split(frame, 2, 1)
        grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

        # Stereo
        stereo = cv2.StereoBM(cv2.STEREO_BM_BASIC_PRESET,
                              ndisparities=16, SADWindowSize=15)
        disparity = stereo.compute(grayR, grayL)

        # Display the resulting frame
        # cv2.imshow('right', imgR)
        cv2.imshow('right', grayR)
        cv2.imshow('left', imgL)
        cv2.imshow('disparity', disparity)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    t1()
