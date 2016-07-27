#!/usr/bin/env python

import cv2
import cv_bridge
import rospy
import os
from sensor_msgs.msg import Image
from find_ball_position_in_img import get_pos
from geometry_msgs.msg import PoseStamped
from math import cos, radians, sin, tan
import numpy as np


def get_ball_pos(msg):
    global x, y, radius, img, res, mask, masked, masked2
    img = bridge.imgmsg_to_cv2(msg)
    res = cv2.resize(img, None, fx=1, fy=1, interpolation=cv2.INTER_AREA)
    masked = cv2.bitwise_and(res, res, mask=mask)
    # masked2 = masked.copy()

    # print img.shape
    x, y, radius = get_pos(masked)
    print '{}, {} on img'.format(x, y)
    # x2, y2 = get_pos(masked2, hmin=0, smin=135, vmin=0,
    #                  hmax=18, smax=255, vmax=255)
    # print img.shape
    # print '{} et {}'.format(x, y)


def get_ball_depth(msg):
    global x, y, z, xr, yr, res, radius
    if x == -1 and y == -1:
        xr = -1
        yr = -1
    else:
        img = bridge.imgmsg_to_cv2(msg)
        tmp = cv2.resize(img, None, fx=1, fy=1,
                         interpolation=cv2.INTER_AREA)
        z = tmp[y, x]
        # print img.shape
        # print z, x, y
        if z in [np.inf, np.NaN] or np.isnan(z):
            print 'depth is nan',
            print 'setting to estimated value from radius', radius,
            a_ball = (110. / img.shape[0]) * (radius)
            print a_ball
            z = 0.1 / tan(radians(a_ball))
        alpha = (110. / img.shape[0]) * (x - tmp.shape[1] / 2.)
        xr = z * sin(radians(alpha))
        yr = z * cos(radians(alpha))
    print 'ball found at ({},{}), {}'.format(xr, yr, z)
        # print "{:<20.3f}{:<20.3f}{:<20.3f}{:<20.3f}".format(alpha, xr, yr, z)

rospy.init_node('pose_ball_publisher')

# Subscribers
img_sub = rospy.Subscriber(
    '/camera/left/image_rect_color', Image, get_ball_pos)

depth_sub = rospy.Subscriber(
    '/camera/depth/image_rect_color', Image, get_ball_depth)

pub = rospy.Publisher('ball/pose', PoseStamped, queue_size=1)

# Bridges
bridge = cv_bridge.CvBridge()
# img = cv2.imread('../data/left0004.jpg')
img = np.zeros((376, 672, 3), np.uint8)
res = np.zeros((376, 672, 3), np.uint8)
path = '/'.join(__file__.split('/')[:-2]) + '/data/mask0000.jpg'
mask = cv2.imread(path, 0)
masked = np.zeros((376, 672, 3), np.uint8)
# masked2 = np.zeros((376, 672, 3), np.uint8)

# Variables
x, y, z, radius = -1, -1, -1, -1
xr, yr = -1, -1

# Parameter
rate = rospy.Rate(10)
show_img = rospy.get_param("~show_img", True)

while True:
    # pub.
    # print xr, yr
    po = PoseStamped()
    po.header.stamp = rospy.Time.now()
    po.header.frame_id = 'boat_frame'
    po.pose.position.x = xr
    po.pose.position.y = yr
    pub.publish(po)
    if show_img:
        # cv2.imshow('frame', res)
        cv2.imshow('masked', masked)
        # cv2.imshow('masked2', masked2)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    rate.sleep()
