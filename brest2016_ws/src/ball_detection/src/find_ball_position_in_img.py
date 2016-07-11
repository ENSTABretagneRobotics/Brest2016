#!/usr/bin/env python
import cv2


def get_pos(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Lower and upper Orange color for the ball
    orangeLower = (0, 219, 99)
    orangeUpper = (18, 255, 255)

    mask = cv2.inRange(hsv, orangeLower, orangeUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            # print '{}({}), {}({})'.format(x, int(x), y, int(y))
            # print depth[y, x]
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            return x, y
    return -1, -1

if __name__ == '__main__':
    # WEBCAM
    # camera = cv2.VideoCapture(0)

    # Read image
    frame = cv2.imread('../data/left0003.jpg')
    depth = cv2.imread('../data/depth.jpg')
    # print 'frame {}, depth {}'.format(frame.shape, depth.shape)
    while True:
        cv2.imshow('depth', depth)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    #  Blur it
    frame = cv2.GaussianBlur(frame, (11, 11), 0)
    # convert it to the HSV

    get_pos(frame)
    while True:
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
