#!/usr/bin/env  python
import cv2

# WEBCAM
camera = cv2.VideoCapture(0)

# DATA IMAGES
ball_close_img = cv2.imread('../data/ball_close.jpg')
ball_mid_img = cv2.imread('../data/ball_mid.jpg')
ball_far_img = cv2.imread('../data/ball_far.jpg')


def nothing(x):
    pass


def create_trackbars():
    cv2.namedWindow('HSV calibration')
    cv2.createTrackbar('hmin', 'HSV calibration', 0, 255, nothing)
    cv2.createTrackbar('hmax', 'HSV calibration', 0, 255, nothing)
    cv2.createTrackbar('smin', 'HSV calibration', 0, 255, nothing)
    cv2.createTrackbar('smax', 'HSV calibration', 0, 255, nothing)
    cv2.createTrackbar('vmin', 'HSV calibration', 0, 255, nothing)
    cv2.createTrackbar('vmax', 'HSV calibration', 0, 255, nothing)


def get_trackbars_values():
    hmin = cv2.getTrackbarPos('hmin', 'HSV calibration')
    hmax = cv2.getTrackbarPos('hmax', 'HSV calibration')
    smin = cv2.getTrackbarPos('smin', 'HSV calibration')
    smax = cv2.getTrackbarPos('smax', 'HSV calibration')
    vmin = cv2.getTrackbarPos('vmin', 'HSV calibration')
    vmax = cv2.getTrackbarPos('vmax', 'HSV calibration')
    return [hmin, hmax, smin, smax, vmin, vmax]


create_trackbars()
# keep looping
while True:
    # grab the current frame
    # (grabbed, frame) = camera.read()

    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
    # if not grabbed:
    #     break

    # resize the frame
    # frame = cv2.resize(frame, (600, 600))

    frame = ball_far_img
    #  Blur it
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    # convert it to the HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # split channels
    h, s, v = cv2.split(frame)

    # For each layer
    # construct a mask for the color "orange", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask

    hmin, hmax, smin, smax, vmin, vmax = get_trackbars_values()
    h_mask = cv2.inRange(h, hmin, hmax)
    s_mask = cv2.inRange(s, smin, smax)
    v_mask = cv2.inRange(v, vmin, vmax)

    # Final
    orangeLower = (hmin, smin, vmin)
    orangeUpper = (hmax, smax, vmax)
    mask = cv2.inRange(hsv, orangeLower, orangeUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # cv2.imshow('hsv', hsv)
    cv2.imshow('original', frame)
    cv2.imshow('h_mask', h_mask)
    cv2.imshow('s_mask', s_mask)
    cv2.imshow('v_mask', v_mask)
    cv2.imshow('final', mask)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
