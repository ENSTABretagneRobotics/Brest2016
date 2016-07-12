#!/usr/bin/env python

import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from find_ball_position_in_img import get_pos
from geometry_msgs.msg import PoseStamped
from math import cos, radians, sin
import numpy as np


def get_ball_pos(msg):
    global x, y, img, res
    img = bridge.imgmsg_to_cv2(msg)
    res = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    # print img.shape
    x, y = get_pos(res)
    # print img.shape
    # print '{} et {}'.format(x, y)


def get_ball_depth(msg):
    global x, y, z, xr, yr, res
    if x == -1 and y == -1:
        xr = -1
        yr = -1
    else:
        img = bridge.imgmsg_to_cv2(msg)
        tmp = cv2.resize(img, None, fx=0.5, fy=0.5,
                         interpolation=cv2.INTER_AREA)
        z = tmp[y, x]
        # print img.shape
        # print z, x, y
        alpha = (110. / img.shape[0]) * (x - tmp.shape[1] / 2.)
        xr = z * sin(radians(alpha))
        yr = z * cos(radians(alpha))
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

# Variables
x, y, z = -1, -1, -1
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
        cv2.imshow('frame', res)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    rate.sleep()
