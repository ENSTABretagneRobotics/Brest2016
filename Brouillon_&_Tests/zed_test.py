#!/usr/bin/env python
import cv2
import numpy as np


def t1():
    cap = cv2.VideoCapture(1)
    i = 0

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Our operations on the frame come here
        imgL, imgR = np.array_split(frame, 2, 1)
        resL = cv2.resize(imgL, None, fx=0.25, fy=0.25,
                          interpolation=cv2.INTER_AREA)
        resR = cv2.resize(imgR, None, fx=0.25, fy=0.25,
                          interpolation=cv2.INTER_AREA)
        # grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
        # grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)cc

        # Stereo
        # stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
        # disparity = stereo.compute(grayR, grayL)

        # Display the resulting frame
        cv2.imshow('right', resR)
        # cv2.imshow('right', grayR)
        cv2.imshow('left', resL)
        # cv2.imshow('disparity', disparity)q
        if cv2.waitKey(1) & 0xFF == ord('p'):
            cv2.imwrite('zed_picL.jpg', imgL)
            cv2.imwrite('zed_picR.jpg', imgR)
        if cv2.waitKey(1) & 0xFF == ord('c'):
            cv2.imwrite('zed_picL_cal' + str(i) + '.jpg', imgL)
            cv2.imwrite('zed_picR_cal' + str(i) + '.jpg', imgR)
            print 'chessboard', i
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    t1()
