#!/usr/bin/env python
import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from find_ball_position_in_img import get_pos
from math import atan, degrees


def get_ball_pos(msg):
    global x, y, img
    img = bridge.imgmsg_to_cv2(msg)
    x, y = get_pos(img)
    print '{} et {}'.format(x, y)


def get_ball_depth(msg):
    global x, y, z
    img = bridge.imgmsg_to_cv2(msg)
    z = img[y, x]
    print z, img.shape[0] - y, x, degrees(atan((img.shape[0] - y) / (x - 640 / 2)))

rospy.init_node('distance_ball')

img_sub = rospy.Subscriber(
    '/camera/left/image_rect_color', Image, get_ball_pos)
depth_sub = rospy.Subscriber(
    '/camera/depth/image_rect_color', Image, get_ball_depth)

bridge = cv_bridge.CvBridge()
img = cv2.imread('../data/left0000.jpg')

x, y, z = -1, -1, -1
rate = rospy.Rate(10)
while True:
    rate.sleep()
    cv2.imshow('frame', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
