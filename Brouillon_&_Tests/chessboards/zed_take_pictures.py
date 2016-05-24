#!/usr/bin/env python
import cv2
import numpy as np
from time import sleep


def t1():
    cap = cv2.VideoCapture(1)

    for i in range(10):
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Our operations on the frame come here
        imgL, imgR = np.array_split(frame, 2, 1)

        # Display the resulting frame
        # cv2.imshow('right', imgR)
        # cv2.imshow('left', imgL)
        # cv2.imshow('disparity', disparity)
        cv2.imwrite('raw/zed_picL_cal' + str(i) + '.jpg', imgL)
        cv2.imwrite('raw/zed_picR_cal' + str(i) + '.jpg', imgR)
        print 'chessboard', i
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        sleep(3)

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    t1()
