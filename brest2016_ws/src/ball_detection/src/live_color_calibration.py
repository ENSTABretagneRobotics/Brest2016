#!/usr/bin/env  python
import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image
import numpy as np


def update_img(msg):
    global frame
    frame = bridge.imgmsg_to_cv2(msg)


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


rospy.init_node('ball_color_calibration')
create_trackbars()

# IMAGES subscriber
img_sub = rospy.Subscriber(
    '/camera/left/image_rect_color', Image, update_img)

# Cv bridge
bridge = cv_bridge.CvBridge()

frame = np.zeros((376, 672, 3), np.uint8)

# keep looping
while True:

    # # resize the frame to process faster !
    # frame = cv2.resize(frame, (600, 600))

    # frame = real_ball_img
    #  Blur it
    # frame = cv2.GaussianBlur(frame, (11, 11), 0)
    # convert it to the HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # split channels
    h, s, v = cv2.split(frame)

    # For each layer
    # construct a mask for the color "orange", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask

    hmin, hmax, smin, smax, vmin, vmax = get_trackbars_values()
    #                                    # Best values found:
    h_mask = cv2.inRange(h, hmin, hmax)  # 0 2
    s_mask = cv2.inRange(s, smin, smax)  # 0 22
    v_mask = cv2.inRange(v, vmin, vmax)  # 75 255

    # Final
    #                                   # Best values found:
    orangeLower = (hmin, smin, vmin)    # 0, 176, 72
    orangeUpper = (hmax, smax, vmax)    # 16, 255, 255
    mask = cv2.inRange(hsv, orangeLower, orangeUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # cv2.imshow('hsv', hsv)
    cv2.imshow('original', frame)
    # cv2.imshow('h_mask', h_mask)
    # cv2.imshow('s_mask', s_mask)
    # cv2.imshow('v_mask', v_mask)
    cv2.imshow('final', mask)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
