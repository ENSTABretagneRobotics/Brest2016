#!/usr/bin/env python
import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from find_ball_position_in_img import get_pos
from geometry_msgs.msg import PoseStamped
from math import cos, radians, sin


def get_ball_pos(msg):
    global x, y, img, res
    img = bridge.imgmsg_to_cv2(msg)
    # print img.shape
    res = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    x, y = get_pos(res)
    # print img.shape
    # print '{} et {}'.format(x, y)


def get_ball_depth(msg):
    global x, y, z, xr, yr, res
    img = bridge.imgmsg_to_cv2(msg)
    tmp = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    z = tmp[y, x]
    # print img.shape
    # print x, y, z
    alpha = 1280 / 110 * (x - 640)
    xr = z * sin(radians(alpha))
    yr = z * cos(radians(alpha))

rospy.init_node('distance_ball')

img_sub = rospy.Subscriber(
    '/camera/left/image_rect_color', Image, get_ball_pos)
depth_sub = rospy.Subscriber(
    '/camera/depth/image_rect_color', Image, get_ball_depth)

bridge = cv_bridge.CvBridge()
img = cv2.imread('../data/left0004.jpg')
res = cv2.imread('../data/left0004.jpg')

pub = rospy.Publisher('ball_pose_finder', PoseStamped, queue_size=1)

x, y, z = -1, -1, -1
xr, yr = -1, -1
rate = rospy.Rate(10)
while True:
    rate.sleep()
    # pub.
    print xr, yr
    po = PoseStamped()
    po.header.stamp = rospy.Time.now()
    po.header.frame_id = 'boat_frame'
    po.pose.position.x = xr
    po.pose.position.y = yr
    pub.publish(po)
    cv2.imshow('frame', res)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
